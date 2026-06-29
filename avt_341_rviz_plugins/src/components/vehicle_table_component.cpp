#include <avt_341_rviz_plugins/components/vehicle_table_component.h>

#include <QAbstractItemView>
#include <QBrush>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QSize>
#include <QStackedLayout>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>

#include <avt_341_rviz_plugins/primitives/icon_button.h>
#include <avt_341_rviz_plugins/primitives/message_label.h>
#include <avt_341_rviz_plugins/primitives/vehicle_palette.h>

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

// The Compute-cell icon: a success or error glyph, rendered at a small cell
// size scaled to the display's DPI.
QPixmap computePixmap( bool healthy )
{
    const QString file = healthy ? "msg_success.svg" : "msg_error.svg";
    return avt_341::rviz_plugins::renderSvg( file, 18 );
}

// Side length of the color square shown beside a vehicle id.
constexpr int kSwatchSize = 12;

// Modal dialog to enter a vehicle name and pick a label color from the palette.
// Returns true if accepted, writing the entered name and the chosen palette
// index. `initial_color_index` pre-selects a palette color (wrapped into range).
bool promptVehicle( QWidget* parent, const QString& title, const QString& initial_name,
                    int initial_color_index, QString* out_name, int* out_color_index )
{
    using namespace avt_341::rviz_plugins;

    QDialog dialog( parent );
    dialog.setWindowTitle( title );

    QLineEdit* name_edit = new QLineEdit( initial_name, &dialog );
    name_edit->setPlaceholderText( "Vehicle name" );

    // Color picker: one entry per palette color, each a swatch icon + its name.
    QComboBox* color_combo = new QComboBox( &dialog );
    const int swatch_size = scaledSize( kSwatchSize, parent );
    color_combo->setIconSize( QSize( swatch_size, swatch_size ) );
    const QVector<VehicleColor>& palette = vehiclePalette();
    for ( const VehicleColor& entry : palette )
    {
        color_combo->addItem( QIcon( makeColorSwatch( entry.color, swatch_size ) ), entry.name );
    }
    const int n = palette.size();
    color_combo->setCurrentIndex( ( ( initial_color_index % n ) + n ) % n );

    QFormLayout* form = new QFormLayout;
    form->addRow( "Name:", name_edit );
    form->addRow( "Color:", color_combo );

    QDialogButtonBox* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog );
    QObject::connect( buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept );
    QObject::connect( buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject );

    QVBoxLayout* layout = new QVBoxLayout( &dialog );
    layout->addLayout( form );
    layout->addWidget( buttons );

    name_edit->setFocus();
    name_edit->selectAll();

    if ( dialog.exec() != QDialog::Accepted )
    {
        return false;
    }

    *out_name = name_edit->text();
    *out_color_index = color_combo->currentIndex();
    return true;
}

}  // namespace

namespace avt_341::rviz_plugins
{

VehicleTableComponent::VehicleTableComponent( QWidget* parent )
    : QWidget( parent )
{
    // Icon-only controls, stacked in a vertical strip (mirrors the old list).
    add_button_ = new IconButton( "add.svg", "Add" );
    delete_button_ = new IconButton( "delete.svg", "Delete" );
    edit_button_ = new IconButton( "edit.svg", "Edit" );
    up_button_ = new IconButton( "arrow_up.svg", "Move Up" );
    down_button_ = new IconButton( "arrow_down.svg", "Move Down" );

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
    // Size the cell icons to the vehicle-id color swatch (scaled to the display
    // DPI) so it renders crisply.
    const int swatch_size = scaledSize( kSwatchSize, table_ );
    table_->setIconSize( QSize( swatch_size, swatch_size ) );
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

    // Vehicle Id: a color square (the vehicle's label color) beside the id text.
    QTableWidgetItem* id_item = new QTableWidgetItem( vehicle_id );
    id_item->setIcon( QIcon( makeColorSwatch( colorForRow( row ), scaledSize( kSwatchSize, table_ ) ) ) );
    table_->setItem( row, kVehicleIdColumn, id_item );

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

QColor VehicleTableComponent::colorForRow( int row ) const
{
    const int stored = status_.value( vehicle_ids_.at( row ) ).color_index;
    // -1 means "auto": fall back to the row position so the default color tracks
    // the modulo lookup; an explicit selection overrides it.
    return vehicleColorForIndex( stored >= 0 ? stored : row );
}

int VehicleTableComponent::rowOf( const QString& vehicle_id ) const
{
    return vehicle_ids_.indexOf( vehicle_id );
}

QColor VehicleTableComponent::vehicleColor( const QString& vehicle_id ) const
{
    const int row = rowOf( vehicle_id );
    return row < 0 ? vehicleColorForIndex( 0 ) : colorForRow( row );
}

QMap<QString, int> VehicleTableComponent::colorIndices() const
{
    // Resolve every vehicle to a concrete palette index so colors survive a
    // reload and a later reorder keeps each vehicle's color.
    QMap<QString, int> indices;
    const int n = vehiclePalette().size();
    for ( int row = 0; row < vehicle_ids_.size(); ++row )
    {
        const int stored = status_.value( vehicle_ids_.at( row ) ).color_index;
        const int resolved = stored >= 0 ? stored : row;
        indices.insert( vehicle_ids_.at( row ), ( ( resolved % n ) + n ) % n );
    }
    return indices;
}

void VehicleTableComponent::setColorIndices( const QMap<QString, int>& indices )
{
    for ( auto it = indices.constBegin(); it != indices.constEnd(); ++it )
    {
        status_[it.key()].color_index = it.value();
    }
    renderTable();
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
    // Default the color to the new vehicle's position so a fresh list walks the
    // palette (red, blue, green, ...); the picker lets the user override it.
    QString name;
    int color_index = vehicle_ids_.size();
    if ( !promptVehicle( this, "Add Vehicle", QString(), color_index, &name, &color_index ) )
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

    VehicleStatus status;
    status.color_index = color_index;
    vehicle_ids_.append( trimmed );
    status_.insert( trimmed, status );
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
    // Pre-select the picker with the vehicle's currently displayed color (its
    // explicit selection, or the position-based default when on "auto").
    const int stored_index = status_.value( old_id ).color_index;
    const int current_index = stored_index >= 0 ? stored_index : row;

    QString name;
    int color_index = current_index;
    if ( !promptVehicle( this, "Edit Vehicle", old_id, current_index, &name, &color_index ) )
    {
        return;
    }

    const QString trimmed = name.trimmed();
    if ( trimmed.isEmpty() )
    {
        return;
    }

    const bool name_changed = ( trimmed != old_id );
    if ( name_changed && isDuplicate( trimmed, row ) )
    {
        QMessageBox::warning(
            this, "Duplicate Vehicle",
            QString( "A vehicle named \"%1\" already exists." ).arg( trimmed ) );
        return;
    }

    // Carry the cached status across the (possible) rename and apply the picked
    // color. Bail out early only if nothing actually changed.
    const bool color_changed = ( color_index != current_index );
    if ( !name_changed && !color_changed )
    {
        return;
    }

    VehicleStatus status = status_.value( old_id );
    status.color_index = color_index;
    if ( name_changed )
    {
        status_.remove( old_id );
        vehicle_ids_[row] = trimmed;
    }
    status_.insert( vehicle_ids_.at( row ), status );
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
