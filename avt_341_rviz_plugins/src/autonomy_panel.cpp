#include <avt_341_rviz_plugins/autonomy_panel.h>

#include <QGroupBox>
#include <QLayoutItem>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <avt_341_rviz_plugins/components/setup_component.h>
#include <avt_341_rviz_plugins/components/nav_state_component.h>
#include <avt_341_rviz_plugins/components/mission_component.h>
#include <avt_341_rviz_plugins/components/tracker_component.h>
#include <avt_341_rviz_plugins/components/compute_component.h>

#include <avt_341_rviz_plugins/primitives/message_label.h>

namespace avt_341::rviz_plugins
{

AutonomyPanel::AutonomyPanel( QWidget* parent )
    : rviz_common::Panel( parent )
{
    // Setup tab: manages the list of vehicles
    setup_component_ = new SetupComponent();

    // Per-vehicle tabs, populated dynamically as vehicles are added / removed
    QWidget* nav_state_tab = createVehicleTab( nav_state_layout_ );
    QWidget* mission_tab = createVehicleTab( mission_layout_ );
    QWidget* tracker_tab = createVehicleTab( tracker_layout_ );
    QWidget* compute_tab = createVehicleTab( compute_layout_ );

    // Create the top-aligned tab widget and add each tab
    tab_widget_ = new QTabWidget();
    tab_widget_->setTabPosition( QTabWidget::North );
    tab_widget_->addTab( setup_component_, "Setup" );
    tab_widget_->addTab( nav_state_tab,    "Nav State" );
    tab_widget_->addTab( mission_tab,      "Mission" );
    tab_widget_->addTab( tracker_tab,      "Tracker" );
    tab_widget_->addTab( compute_tab,      "Compute" );

    // Layout widgets
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget( tab_widget_ );
    setLayout( layout );

    // Keep the per-vehicle tabs in sync with the Setup vehicle list in real time
    connect( setup_component_, SIGNAL( vehiclesChanged( QStringList ) ),
             this, SLOT( onVehiclesChanged( QStringList ) ) );

    // Render the initial empty-state message before any vehicles are added.
    onVehiclesChanged( QStringList() );
}

void AutonomyPanel::onInitialize()
{
    // Initialize ROS node
    node_ = std::make_shared<rclcpp::Node>("autonomy_panel_node");
}

QWidget* AutonomyPanel::createVehicleTab( QVBoxLayout*& out_content_layout )
{
    // The content widget holds one group box per vehicle, stacked vertically.
    QWidget* content = new QWidget();
    out_content_layout = new QVBoxLayout( content );

    // Wrap in a scroll area so many vehicles don't overflow the panel.
    QScrollArea* scroll = new QScrollArea();
    scroll->setWidgetResizable( true );
    scroll->setWidget( content );
    return scroll;
}

void AutonomyPanel::rebuildVehicleTab(
    QVBoxLayout* content_layout, const QStringList& vehicles,
    const std::function<QWidget*( const QString& )>& make_component )
{
    // Remove existing group boxes (and any trailing stretch).
    QLayoutItem* item;
    while ( ( item = content_layout->takeAt( 0 ) ) != nullptr )
    {
        if ( item->widget() != nullptr )
        {
            item->widget()->deleteLater();
        }
        delete item;
    }

    // Empty state: a centered info message pointing the user to the Setup tab.
    if ( vehicles.isEmpty() )
    {
        content_layout->addStretch();
        content_layout->addWidget(
            new MessageLabel( MessageType::Info, "No vehicles added" ),
            0, Qt::AlignHCenter );
        content_layout->addStretch();
        return;
    }

    // One titled group box per vehicle, wrapping that vehicle's component.
    for ( const QString& vehicle_id : vehicles )
    {
        QGroupBox* group_box = new QGroupBox( vehicle_id );
        QVBoxLayout* group_layout = new QVBoxLayout;
        group_layout->addWidget( make_component( vehicle_id ) );
        group_box->setLayout( group_layout );
        content_layout->addWidget( group_box );
    }

    // Keep the group boxes top-aligned.
    content_layout->addStretch();
}

void AutonomyPanel::onVehiclesChanged( const QStringList& vehicles )
{
    rebuildVehicleTab( nav_state_layout_, vehicles,
                       []( const QString& id ) { return new NavStateComponent( id ); } );
    rebuildVehicleTab( mission_layout_, vehicles,
                       []( const QString& id ) { return new MissionComponent( id ); } );
    rebuildVehicleTab( tracker_layout_, vehicles,
                       []( const QString& id ) { return new TrackerComponent( id ); } );
    rebuildVehicleTab( compute_layout_, vehicles,
                       []( const QString& id ) { return new ComputeComponent( id ); } );
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(avt_341::rviz_plugins::AutonomyPanel, rviz_common::Panel )
