#include <avt_341_rviz_plugins/components/entity_list_component.h>

#include <QAbstractItemView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLineEdit>
#include <QListView>
#include <QMessageBox>
#include <QModelIndex>
#include <QStackedLayout>
#include <QStringListModel>
#include <QVBoxLayout>

#include <avt_341_rviz_plugins/primitives/icon_button.h>
#include <avt_341_rviz_plugins/primitives/message_label.h>

namespace avt_341::rviz_plugins
{

EntityListComponent::EntityListComponent( const QString& title, const QString& item_noun,
                                          QWidget* parent )
    : QWidget( parent ), item_noun_( item_noun )
{
    // Icon-only controls, stacked in a vertical strip
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

    // Selectable, non-editable list of entries
    model_ = new QStringListModel( this );
    list_view_ = new QListView();
    list_view_->setModel( model_ );
    list_view_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    list_view_->setSelectionMode( QAbstractItemView::SingleSelection );

    // Info message centered (vertically and horizontally) over the list view
    // while it holds no entries.
    MessageLabel* empty_message = new MessageLabel(
        MessageType::Info,
        QString( "Click + to add a %1." ).arg( item_noun_.toLower() ) );
    empty_message->setAttribute( Qt::WA_TransparentForMouseEvents );

    empty_overlay_ = new QWidget();
    empty_overlay_->setAttribute( Qt::WA_TransparentForMouseEvents );
    QVBoxLayout* empty_layout = new QVBoxLayout( empty_overlay_ );
    empty_layout->setContentsMargins( 0, 0, 0, 0 );
    empty_layout->addStretch();
    empty_layout->addWidget( empty_message, 0, Qt::AlignHCenter );
    empty_layout->addStretch();

    // Stack the empty-state overlay over the list view; the opaque list is
    // raised to the front (hiding the message) once it has entries.
    QWidget* list_container = new QWidget();
    list_stack_ = new QStackedLayout( list_container );
    list_stack_->setStackingMode( QStackedLayout::StackAll );
    list_stack_->addWidget( list_view_ );
    list_stack_->addWidget( empty_overlay_ );

    // Button strip on the left, list (with overlay) filling the remaining width
    QHBoxLayout* content_layout = new QHBoxLayout;
    content_layout->addLayout( button_layout );
    content_layout->addWidget( list_container, 1 );

    QGroupBox* group_box = new QGroupBox( title );
    group_box->setLayout( content_layout );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( group_box );
    setLayout( layout );

    // Wire up the buttons
    connect( add_button_, SIGNAL( clicked() ), this, SLOT( onAdd() ) );
    connect( delete_button_, SIGNAL( clicked() ), this, SLOT( onDelete() ) );
    connect( edit_button_, SIGNAL( clicked() ), this, SLOT( onEdit() ) );
    connect( up_button_, SIGNAL( clicked() ), this, SLOT( onMoveUp() ) );
    connect( down_button_, SIGNAL( clicked() ), this, SLOT( onMoveDown() ) );

    // Toggle the empty-state overlay whenever the entry count changes.
    connect( model_, SIGNAL( rowsInserted( QModelIndex, int, int ) ),
             this, SLOT( updateEmptyMessage() ) );
    connect( model_, SIGNAL( rowsRemoved( QModelIndex, int, int ) ),
             this, SLOT( updateEmptyMessage() ) );
    connect( model_, SIGNAL( modelReset() ), this, SLOT( updateEmptyMessage() ) );
    updateEmptyMessage();
}

QStringList EntityListComponent::items() const
{
    return model_->stringList();
}

bool EntityListComponent::isDuplicate( const QString& name, int ignore_row ) const
{
    const QStringList list = model_->stringList();
    for ( int row = 0; row < list.size(); ++row )
    {
        if ( row != ignore_row && list.at( row ).compare( name, Qt::CaseInsensitive ) == 0 )
        {
            return true;
        }
    }
    return false;
}

void EntityListComponent::onAdd()
{
    bool ok = false;
    const QString name = QInputDialog::getText(
        this,
        QString( "Add %1" ).arg( item_noun_ ),
        QString( "%1 name:" ).arg( item_noun_ ),
        QLineEdit::Normal,
        QString(),
        &ok );

    if ( ok && !name.trimmed().isEmpty() )
    {
        const QString trimmed = name.trimmed();
        if ( isDuplicate( trimmed ) )
        {
            QMessageBox::warning(
                this, QString( "Duplicate %1" ).arg( item_noun_ ),
                QString( "A %1 named \"%2\" already exists." ).arg( item_noun_.toLower(), trimmed ) );
            return;
        }

        const int row = model_->rowCount();
        model_->insertRow( row );
        model_->setData( model_->index( row ), trimmed );
        list_view_->setCurrentIndex( model_->index( row ) );
        Q_EMIT itemsChanged( model_->stringList() );
    }
}

void EntityListComponent::onDelete()
{
    const QModelIndex index = list_view_->currentIndex();
    if ( index.isValid() )
    {
        model_->removeRow( index.row() );
        Q_EMIT itemsChanged( model_->stringList() );
    }
}

void EntityListComponent::onEdit()
{
    const QModelIndex index = list_view_->currentIndex();
    if ( !index.isValid() )
    {
        return;
    }

    bool ok = false;
    const QString name = QInputDialog::getText(
        this,
        QString( "Edit %1" ).arg( item_noun_ ),
        QString( "%1 name:" ).arg( item_noun_ ),
        QLineEdit::Normal,
        index.data().toString(),
        &ok );

    if ( ok && !name.trimmed().isEmpty() )
    {
        const QString trimmed = name.trimmed();
        if ( isDuplicate( trimmed, index.row() ) )
        {
            QMessageBox::warning(
                this, QString( "Duplicate %1" ).arg( item_noun_ ),
                QString( "A %1 named \"%2\" already exists." ).arg( item_noun_.toLower(), trimmed ) );
            return;
        }

        model_->setData( index, trimmed );
        Q_EMIT itemsChanged( model_->stringList() );
    }
}

void EntityListComponent::onMoveUp()
{
    const QModelIndex index = list_view_->currentIndex();
    if ( !index.isValid() || index.row() <= 0 )
    {
        return;
    }

    const int row = index.row();
    QStringList list = model_->stringList();
    list.insert( row - 1, list.takeAt( row ) );
    model_->setStringList( list );
    list_view_->setCurrentIndex( model_->index( row - 1 ) );
    Q_EMIT itemsChanged( model_->stringList() );
}

void EntityListComponent::onMoveDown()
{
    const QModelIndex index = list_view_->currentIndex();
    if ( !index.isValid() || index.row() >= model_->rowCount() - 1 )
    {
        return;
    }

    const int row = index.row();
    QStringList list = model_->stringList();
    list.insert( row + 1, list.takeAt( row ) );
    model_->setStringList( list );
    list_view_->setCurrentIndex( model_->index( row + 1 ) );
    Q_EMIT itemsChanged( model_->stringList() );
}

void EntityListComponent::updateEmptyMessage()
{
    if ( model_->rowCount() == 0 )
    {
        list_stack_->setCurrentWidget( empty_overlay_ );
    }
    else
    {
        list_stack_->setCurrentWidget( list_view_ );
    }
}

}
