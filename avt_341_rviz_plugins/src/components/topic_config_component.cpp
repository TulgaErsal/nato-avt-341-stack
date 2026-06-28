#include <avt_341_rviz_plugins/components/topic_config_component.h>

#include <cstddef>

#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QInputDialog>
#include <QLineEdit>
#include <QPushButton>

#include <rviz_common/load_resource.hpp>

#include <avt_341_rviz_plugins/primitives/icon_button.h>

namespace
{

// Loads an icon from this package's resources/icons folder via an rviz resource
// URL (resolved from the installed share directory at runtime).
QIcon loadIcon( const QString& file_name )
{
    return QIcon( rviz_common::loadPixmap(
        "package://avt_341_rviz_plugins/resources/icons/" + file_name ) );
}

}  // namespace

namespace avt_341::rviz_plugins
{

TopicConfigComponent::TopicConfigComponent( QWidget* parent )
    : QWidget( parent )
{
    const std::vector<TopicDescriptor>& descriptors = topicDescriptors();

    // "<key>: [read-only value] [edit]" rows. QFormLayout gives the shared label
    // column so the value fields line up.
    // Grouping (title + collapse) is provided by the surrounding AccordionGroup,
    // so this widget is just the flush content.
    QFormLayout* form = new QFormLayout;
    form->setContentsMargins( 0, 0, 0, 0 );
    form->setFieldGrowthPolicy( QFormLayout::AllNonFixedFieldsGrow );

    value_fields_.reserve( descriptors.size() );
    for ( std::size_t i = 0; i < descriptors.size(); ++i )
    {
        const TopicDescriptor& descriptor = descriptors[i];

        // The value is shown but not directly editable; it is changed only via
        // the edit popup.
        QLineEdit* value = new QLineEdit( config_.*( descriptor.member ) );
        value->setReadOnly( true );
        value->setCursorPosition( 0 );

        IconButton* edit = new IconButton( loadIcon( "edit.svg" ), "Edit topic" );

        QWidget* field = new QWidget;
        QHBoxLayout* field_layout = new QHBoxLayout( field );
        field_layout->setContentsMargins( 0, 0, 0, 0 );
        field_layout->setSpacing( 4 );
        field_layout->addWidget( value, 1 );
        field_layout->addWidget( edit );

        form->addRow( QString( descriptor.key ) + ":", field );
        value_fields_.push_back( value );

        const int index = static_cast<int>( i );
        connect( edit, &QPushButton::clicked, this, [this, index]() { onEditTopic( index ); } );
    }

    setLayout( form );
}

void TopicConfigComponent::setConfig( const TopicConfig& config )
{
    config_ = config;
    const std::vector<TopicDescriptor>& descriptors = topicDescriptors();
    for ( std::size_t i = 0; i < descriptors.size() && i < value_fields_.size(); ++i )
    {
        value_fields_[i]->setText( config_.*( descriptors[i].member ) );
        value_fields_[i]->setCursorPosition( 0 );
    }
}

void TopicConfigComponent::onEditTopic( int index )
{
    const std::vector<TopicDescriptor>& descriptors = topicDescriptors();
    if ( index < 0 || index >= static_cast<int>( descriptors.size() ) )
    {
        return;
    }
    const TopicDescriptor& descriptor = descriptors[index];
    const QString current = config_.*( descriptor.member );

    bool ok = false;
    const QString text = QInputDialog::getText(
        this,
        "Edit Topic",
        QString( descriptor.key ) + " topic:",
        QLineEdit::Normal,
        current,
        &ok );

    if ( !ok )
    {
        return;
    }
    const QString trimmed = text.trimmed();
    if ( trimmed.isEmpty() || trimmed == current )
    {
        return;
    }

    config_.*( descriptor.member ) = trimmed;
    value_fields_[index]->setText( trimmed );
    value_fields_[index]->setCursorPosition( 0 );
    Q_EMIT topicChanged( descriptor.group );
}

}
