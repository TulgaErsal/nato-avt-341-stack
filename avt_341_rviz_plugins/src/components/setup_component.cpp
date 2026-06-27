#include <avt_341_rviz_plugins/components/setup_component.h>

#include <QPixmap>
#include <QVBoxLayout>

#include <rviz_common/load_resource.hpp>

#include <avt_341_rviz_plugins/components/entity_list_component.h>
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

    // Vehicles: re-usable add/delete-managed list of named entities.
    vehicles_ = new EntityListComponent( "Vehicles", "Vehicle" );

    // Layout: header on top, vehicles group below, content top-aligned.
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget( header_ );
    layout->addSpacing( 12 );
    layout->addWidget( vehicles_ );
    layout->addStretch();
    setLayout( layout );

    // Forward vehicle-list changes so the panel can rebuild the per-vehicle tabs
    connect( vehicles_, SIGNAL( itemsChanged( QStringList ) ),
             this, SIGNAL( vehiclesChanged( QStringList ) ) );
}

}
