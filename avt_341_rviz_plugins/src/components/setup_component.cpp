#include <avt_341_rviz_plugins/components/setup_component.h>

#include <QPixmap>
#include <QVBoxLayout>

#include <rviz_common/load_resource.hpp>

#include <avt_341_rviz_plugins/components/vehicle_table_component.h>
#include <avt_341_rviz_plugins/components/topic_config_component.h>
#include <avt_341_rviz_plugins/primitives/accordion_group.h>
#include <avt_341_rviz_plugins/primitives/icon_header.h>

namespace avt_341::rviz_plugins
{

SetupComponent::SetupComponent( QWidget* parent )
    : QWidget( parent )
{
    // Header: logo image + "Setup" title on one line. The logo is resolved from
    // the installed package share directory via an rviz resource URL.
    const QPixmap logo =
        rviz_common::loadPixmap( "package://avt_341_rviz_plugins/resources/logo/logo.png" );
    header_ = new IconHeader( logo, "AVT-341 Autonomy Stack" );

    // Vehicles: add/delete-managed table showing each vehicle's live status,
    // grouped under a collapsible accordion.
    vehicles_ = new VehicleTableComponent();
    AccordionGroup* vehicles_group = new AccordionGroup( "Vehicle List" );
    vehicles_group->setContentWidget( vehicles_ );

    // Topics: editable list of the topic names the per-vehicle components use.
    topics_ = new TopicConfigComponent();
    AccordionGroup* topics_group = new AccordionGroup( "Topic Configuration" );
    topics_group->setContentWidget( topics_ );

    // Layout: header on top, accordion groups below, content top-aligned.
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget( header_ );
    layout->addSpacing( 12 );
    layout->addWidget( vehicles_group );
    layout->addSpacing( 12 );
    layout->addWidget( topics_group );
    layout->addStretch();
    setLayout( layout );

    // Forward vehicle-list changes so the panel can rebuild the per-vehicle tabs
    connect( vehicles_, SIGNAL( itemsChanged( QStringList ) ),
             this, SIGNAL( vehiclesChanged( QStringList ) ) );

    // Forward topic changes so the panel can re-create the affected components.
    connect( topics_, &TopicConfigComponent::topicChanged,
             this, &SetupComponent::topicConfigChanged );
}

const TopicConfig& SetupComponent::topicConfig() const
{
    return topics_->config();
}

QStringList SetupComponent::vehicles() const
{
    return vehicles_->items();
}

QColor SetupComponent::vehicleColor( const QString& vehicle_id ) const
{
    return vehicles_->vehicleColor( vehicle_id );
}

QMap<QString, int> SetupComponent::vehicleColorIndices() const
{
    return vehicles_->colorIndices();
}

void SetupComponent::setVehicles( const QStringList& vehicles )
{
    vehicles_->setItems( vehicles );
}

void SetupComponent::setVehicleColorIndices( const QMap<QString, int>& indices )
{
    vehicles_->setColorIndices( indices );
}

void SetupComponent::setTopicConfig( const TopicConfig& config )
{
    topics_->setConfig( config );
}

void SetupComponent::setVehicleNavState( const QString& vehicle_id, const QString& text,
                                         const QColor& color )
{
    vehicles_->setVehicleNavState( vehicle_id, text, color );
}

void SetupComponent::setVehicleComputeHealth( const QString& vehicle_id, bool healthy )
{
    vehicles_->setVehicleComputeHealth( vehicle_id, healthy );
}

}
