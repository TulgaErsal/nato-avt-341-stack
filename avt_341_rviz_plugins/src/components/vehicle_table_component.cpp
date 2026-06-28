#include <avt_341_rviz_plugins/components/vehicle_table_component.h>

#include <QAbstractItemView>
#include <QBrush>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QStackedLayout>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <rviz_common/load_resource.hpp>

#include <avt_341_rviz_plugins/primitives/icon_button.h>
#include <avt_341_rviz_plugins/primitives/message_label.h>

namespace
{

// Table column layout.
enum Column
{
    kVehicleIdColumn = 0,
    kNavStateColumn = 1,
    kComputeColumn = 2,
    kColumnCount = 3
};

// Loads an icon from this package's resources/icons folder via an rviz resource
// URL (resolved from the installed share directory at runtime).
QIcon loadIcon( const QString& file_name )
{
    return QIcon( rviz_common::loadPixmap(
        "package://avt_341_rviz_plugins/resources/icons/" + file_name ) );
}

// The Compute-cell icon: a success or error glyph, scaled to a small cell size.
QPixmap computePixmap( bool healthy )
{
    const QString file = healthy ? "msg_success.svg" : "msg_error.svg";
    return rviz_common::loadPixmap(
               "package://avt_341_rviz_plugins/resources/icons/" + file )
        .scaled( 18, 18, Qt::KeepAspectRatio, Qt::SmoothTransformation );
}

}  // namespace

namespace avt_341::rviz_plugins
{

VehicleTableComponent::VehicleTableComponent( QWidget* parent )
    : QWidget( parent )
{
    // Icon-only controls, stacked in a vertical strip (mirrors the old list).
    add_button_ = new IconButton( loadIcon( "add.svg" ), "Add" );
    delete_button_ = new IconButton( loadIcon( "delete.svg" ), "Delete" );
    edit_button_ = new IconButton( loadIcon( "edit.svg" ), "Edit" );
    up_button_ = new IconButton( loadIcon( "arrow_up.svg" ), "Move Up" );
    down_button_ = new IconButton( loadIcon( "arrow_down.svg" ), "Move Down" );

    QVBoxLayout* button_layout = new QVBoxLayout;
    button_layout->setContentsMargins( 0, 0, 0, 0 );
    button_layout->addWidget( add_button_ );
    button_layout->addWidget( delete_button_ );
    button_layout->addWidget( edit_button_ );
    button_layout->addWidget( up_button_ );
    button_layout->addWidget( down_button_ );
    button_layout->addStretch();

    // Read-only, single-row-selectable status table.
    table_ = new QTableWidget( 0, kColumnCount );
    table_->setHorizontalHeaderLabels( { "Vehicle Id", "Nav State", "Compute" } );
    table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    // Read-only status view: items are not selectable (no selection highlight). A
    // row is still made "current" on click so the add/delete/edit/move buttons
    // have a target.
    table_->setSelectionMode( QAbstractItemView::NoSelection );
    table_->setFocusPolicy( Qt::NoFocus );
    table_->verticalHeader()->setVisible( false );
    // Don't bold the header of the current row's column (Qt's current-section
    // highlight); keep the headers a normal weight like the Compute table.
    table_->horizontalHeader()->setHighlightSections( false );
    table_->horizontalHeader()->setSectionResizeMode( kVehicleIdColumn, QHeaderView::Stretch );
    table_->horizontalHeader()->setSectionResizeMode( kNavStateColumn, QHeaderView::Stretch );
    table_->horizontalHeader()->setSectionResizeMode( kComputeColumn, QHeaderView::ResizeToContents );

    // Info message centered over the table while it holds no vehicles.
    MessageLabel* empty_message = new MessageLabel(
        MessageType::Info, "Click + to add a vehicle." );
    empty_message->setAttribute( Qt::WA_TransparentForMouseEvents );

    empty_overlay_ = new QWidget();
    empty_overlay_->setAttribute( Qt::WA_TransparentForMouseEvents );
    QVBoxLayout* empty_layout = new QVBoxLayout( empty_overlay_ );
    empty_layout->setContentsMargins( 0, 0, 0, 0 );
    empty_layout->addStretch();
    empty_layout->addWidget( empty_message, 0, Qt::AlignHCenter );
    empty_layout->addStretch();

    // Stack the empty-state overlay over the table; the current widget is raised
    // to the front, so the (opaque) table hides the message once it has rows.
    QWidget* table_container = new QWidget();
    table_stack_ = new QStackedLayout( table_container );
    table_stack_->setStackingMode( QStackedLayout::StackAll );
    table_stack_->addWidget( table_ );
    table_stack_->addWidget( empty_overlay_ );

    // Button strip on the left, table (with overlay) filling the remaining width.
    // Grouping (title + collapse) is provided by the surrounding AccordionGroup,
    // so this widget is just the flush content.
    QHBoxLayout* content_layout = new QHBoxLayout;
    content_layout->setContentsMargins( 0, 0, 0, 0 );
    content_layout->addLayout( button_layout );
    content_layout->addWidget( table_container, 1 );
    setLayout( content_layout );

    // Wire up the buttons.
    connect( add_button_, SIGNAL( clicked() ), this, SLOT( onAdd() ) );
    connect( delete_button_, SIGNAL( clicked() ), this, SLOT( onDelete() ) );
    connect( edit_button_, SIGNAL( clicked() ), this, SLOT( onEdit() ) );
    connect( up_button_, SIGNAL( clicked() ), this, SLOT( onMoveUp() ) );
    connect( down_button_, SIGNAL( clicked() ), this, SLOT( onMoveDown() ) );

    renderTable();
}

QStringList VehicleTableComponent::items() const
{
    return vehicle_ids_;
}

void VehicleTableComponent::setItems( const QStringList& vehicle_ids )
{
    vehicle_ids_ = vehicle_ids;
    renderTable();
    Q_EMIT itemsChanged( vehicle_ids_ );
}

void VehicleTableComponent::setVehicleNavState( const QString& vehicle_id,
                                                const QString& text, const QColor& color )
{
    const int row = rowOf( vehicle_id );
    if ( row < 0 )
    {
        return;   // unknown vehicle; ignore
    }

    VehicleStatus& status = status_[vehicle_id];
    status.nav_text = text;
    status.nav_color = color;

    if ( QTableWidgetItem* item = table_->item( row, kNavStateColumn ) )
    {
        item->setText( text );
        item->setBackground( QBrush( color ) );
        item->setForeground( QBrush( Qt::white ) );
    }
}

void VehicleTableComponent::setVehicleComputeHealth( const QString& vehicle_id, bool healthy )
{
    const int row = rowOf( vehicle_id );
    if ( row < 0 )
    {
        return;   // unknown vehicle; ignore
    }

    status_[vehicle_id].compute_healthy = healthy;

    if ( QLabel* icon = qobject_cast<QLabel*>( table_->cellWidget( row, kComputeColumn ) ) )
    {
        icon->setPixmap( computePixmap( healthy ) );
    }
}

void VehicleTableComponent::renderTable()
{
    table_->setRowCount( vehicle_ids_.size() );
    for ( int row = 0; row < vehicle_ids_.size(); ++row )
    {
        writeRow( row, vehicle_ids_.at( row ) );
    }

    table_stack_->setCurrentWidget( vehicle_ids_.isEmpty() ? empty_overlay_
                                                           : static_cast<QWidget*>( table_ ) );
}

void VehicleTableComponent::writeRow( int row, const QString& vehicle_id )
{
    const VehicleStatus status = status_.value( vehicle_id );

    table_->setItem( row, kVehicleIdColumn, new QTableWidgetItem( vehicle_id ) );

    // Nav State: colored cell carrying the run-state text.
    QTableWidgetItem* nav_item = new QTableWidgetItem( status.nav_text );
    nav_item->setTextAlignment( Qt::AlignCenter );
    nav_item->setBackground( QBrush( status.nav_color ) );
    nav_item->setForeground( QBrush( Qt::white ) );
    table_->setItem( row, kNavStateColumn, nav_item );

    // Compute: a centered success / error icon. Transparent to mouse events so a
    // click still selects the row.
    QLabel* icon = new QLabel();
    icon->setAlignment( Qt::AlignCenter );
    icon->setAttribute( Qt::WA_TransparentForMouseEvents );
    icon->setPixmap( computePixmap( status.compute_healthy ) );
    table_->setCellWidget( row, kComputeColumn, icon );
}

int VehicleTableComponent::rowOf( const QString& vehicle_id ) const
{
    return vehicle_ids_.indexOf( vehicle_id );
}

bool VehicleTableComponent::isDuplicate( const QString& vehicle_id, int ignore_row ) const
{
    for ( int row = 0; row < vehicle_ids_.size(); ++row )
    {
        if ( row != ignore_row &&
             vehicle_ids_.at( row ).compare( vehicle_id, Qt::CaseInsensitive ) == 0 )
        {
            return true;
        }
    }
    return false;
}

void VehicleTableComponent::onAdd()
{
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, "Add Vehicle", "Vehicle name:", QLineEdit::Normal, QString(), &ok );
    if ( !ok )
    {
        return;
    }

    const QString trimmed = name.trimmed();
    if ( trimmed.isEmpty() )
    {
        return;
    }
    if ( isDuplicate( trimmed ) )
    {
        QMessageBox::warning(
            this, "Duplicate Vehicle",
            QString( "A vehicle named \"%1\" already exists." ).arg( trimmed ) );
        return;
    }

    vehicle_ids_.append( trimmed );
    status_.insert( trimmed, VehicleStatus() );
    renderTable();
    table_->setCurrentCell( vehicle_ids_.size() - 1, kVehicleIdColumn );
    Q_EMIT itemsChanged( vehicle_ids_ );
}

void VehicleTableComponent::onDelete()
{
    const int row = table_->currentRow();
    if ( row < 0 || row >= vehicle_ids_.size() )
    {
        return;
    }

    const QString id = vehicle_ids_.at( row );
    vehicle_ids_.removeAt( row );
    status_.remove( id );
    renderTable();
    Q_EMIT itemsChanged( vehicle_ids_ );
}

void VehicleTableComponent::onEdit()
{
    const int row = table_->currentRow();
    if ( row < 0 || row >= vehicle_ids_.size() )
    {
        return;
    }

    const QString old_id = vehicle_ids_.at( row );
    bool ok = false;
    const QString name = QInputDialog::getText(
        this, "Edit Vehicle", "Vehicle name:", QLineEdit::Normal, old_id, &ok );
    if ( !ok )
    {
        return;
    }

    const QString trimmed = name.trimmed();
    if ( trimmed.isEmpty() || trimmed == old_id )
    {
        return;
    }
    if ( isDuplicate( trimmed, row ) )
    {
        QMessageBox::warning(
            this, "Duplicate Vehicle",
            QString( "A vehicle named \"%1\" already exists." ).arg( trimmed ) );
        return;
    }

    // Carry the cached status across the rename.
    vehicle_ids_[row] = trimmed;
    const VehicleStatus status = status_.value( old_id );
    status_.remove( old_id );
    status_.insert( trimmed, status );
    renderTable();
    table_->setCurrentCell( row, kVehicleIdColumn );
    Q_EMIT itemsChanged( vehicle_ids_ );
}

void VehicleTableComponent::onMoveUp()
{
    const int row = table_->currentRow();
    if ( row <= 0 || row >= vehicle_ids_.size() )
    {
        return;
    }

    vehicle_ids_.move( row, row - 1 );
    renderTable();
    table_->setCurrentCell( row - 1, kVehicleIdColumn );
    Q_EMIT itemsChanged( vehicle_ids_ );
}

void VehicleTableComponent::onMoveDown()
{
    const int row = table_->currentRow();
    if ( row < 0 || row >= vehicle_ids_.size() - 1 )
    {
        return;
    }

    vehicle_ids_.move( row, row + 1 );
    renderTable();
    table_->setCurrentCell( row + 1, kVehicleIdColumn );
    Q_EMIT itemsChanged( vehicle_ids_ );
}

}
