#include <avt_341_rviz_plugins/panel_plugins/autonomy_panel.h>

#include <rviz_common/config.hpp>

#include <QLayoutItem>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWidget>

#include <avt_341_rviz_plugins/primitives/accordion_group.h>

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

    // Layout widgets. Drop the layout's default margins so the tab control sits
    // flush with the panel's bounds.
    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( tab_widget_ );
    setLayout( layout );

    // Keep the per-vehicle tabs in sync with the Setup vehicle list in real time
    connect( setup_component_, SIGNAL( vehiclesChanged( QStringList ) ),
             this, SLOT( onVehiclesChanged( QStringList ) ) );

    // Re-create the affected components whenever a topic name is edited.
    connect( setup_component_, &SetupComponent::topicConfigChanged,
             this, &AutonomyPanel::onTopicConfigChanged );

    // Render the initial empty-state message before any vehicles are added.
    onVehiclesChanged( QStringList() );
}

void AutonomyPanel::onInitialize()
{
    // Initialize ROS node
    node_ = std::make_shared<rclcpp::Node>("autonomy_panel_node");

    // Pump the node from the Qt event loop. Spinning on the UI thread keeps all
    // subscription callbacks on the same thread as the widgets they update, so
    // components can refresh themselves without extra synchronization.
    executor_.add_node( node_ );
    spin_timer_ = new QTimer( this );
    connect( spin_timer_, &QTimer::timeout, this, [this]() { executor_.spin_some(); } );
    spin_timer_->start( 10 );
}

void AutonomyPanel::save( rviz_common::Config config ) const
{
    rviz_common::Panel::save( config );

    // 1) Vehicle list (order preserved).
    rviz_common::Config vehicles = config.mapMakeChild( "vehicles" );
    for ( const QString& vehicle : setup_component_->vehicles() )
    {
        vehicles.listAppendNew().setValue( vehicle );
    }

    // 1b) Per-vehicle label colors (palette index keyed by id).
    rviz_common::Config vehicle_colors = config.mapMakeChild( "vehicle_colors" );
    const QMap<QString, int> color_indices = setup_component_->vehicleColorIndices();
    for ( auto it = color_indices.constBegin(); it != color_indices.constEnd(); ++it )
    {
        rviz_common::Config entry = vehicle_colors.listAppendNew();
        entry.mapSetValue( "id", it.key() );
        entry.mapSetValue( "color", it.value() );
    }

    // 2) Topic-name configuration.
    rviz_common::Config topics = config.mapMakeChild( "topic_config" );
    const TopicConfig& topic_config = setup_component_->topicConfig();
    for ( const TopicDescriptor& descriptor : topicDescriptors() )
    {
        topics.mapSetValue( descriptor.key, topic_config.*( descriptor.member ) );
    }

    // 3) Compute monitored topics (shared across all vehicles) + alert threshold.
    rviz_common::Config compute_topics = config.mapMakeChild( "compute_topics" );
    for ( const ComputeComponent::MonitoredTopicSpec& spec :
          ComputeComponent::monitoredTopics() )
    {
        rviz_common::Config entry = compute_topics.listAppendNew();
        entry.mapSetValue( "topic", spec.suffix );
        entry.mapSetValue( "expected_hz", spec.expected_hz );
    }
    config.mapSetValue( "compute_threshold", ComputeComponent::thresholdFraction() );
}

void AutonomyPanel::load( const rviz_common::Config& config )
{
    rviz_common::Panel::load( config );

    // Apply the topic and compute configs before the vehicle list, so the
    // per-vehicle components rebuilt by the vehicle restore read the new values.

    // 2) Topic-name configuration.
    TopicConfig topic_config = setup_component_->topicConfig();
    const rviz_common::Config topics = config.mapGetChild( "topic_config" );
    for ( const TopicDescriptor& descriptor : topicDescriptors() )
    {
        QString value;
        if ( topics.mapGetString( descriptor.key, &value ) && !value.trimmed().isEmpty() )
        {
            topic_config.*( descriptor.member ) = value.trimmed();
        }
    }
    setup_component_->setTopicConfig( topic_config );

    // 3) Compute monitored topics + threshold (a present list, even empty, wins).
    const rviz_common::Config compute_topics = config.mapGetChild( "compute_topics" );
    if ( compute_topics.getType() == rviz_common::Config::List )
    {
        std::vector<ComputeComponent::MonitoredTopicSpec> specs;
        for ( int i = 0; i < compute_topics.listLength(); ++i )
        {
            const rviz_common::Config entry = compute_topics.listChildAt( i );
            QString suffix;
            float hz = 0.0f;
            entry.mapGetString( "topic", &suffix );
            entry.mapGetFloat( "expected_hz", &hz );
            if ( !suffix.trimmed().isEmpty() )
            {
                specs.push_back( { suffix.trimmed(), static_cast<double>( hz ) } );
            }
        }

        float threshold = static_cast<float>( ComputeComponent::thresholdFraction() );
        config.mapGetFloat( "compute_threshold", &threshold );
        ComputeComponent::applyGlobalConfig( specs, static_cast<double>( threshold ) );
    }

    // 1b) Per-vehicle label colors. Applied before the vehicle list so the tab
    // rebuild triggered by setVehicles() tags each accordion with its color.
    const rviz_common::Config vehicle_colors = config.mapGetChild( "vehicle_colors" );
    if ( vehicle_colors.getType() == rviz_common::Config::List )
    {
        QMap<QString, int> color_indices;
        for ( int i = 0; i < vehicle_colors.listLength(); ++i )
        {
            const rviz_common::Config entry = vehicle_colors.listChildAt( i );
            QString id;
            int color = 0;
            entry.mapGetString( "id", &id );
            entry.mapGetInt( "color", &color );
            if ( !id.trimmed().isEmpty() )
            {
                color_indices.insert( id.trimmed(), color );
            }
        }
        setup_component_->setVehicleColorIndices( color_indices );
    }

    // 1) Vehicle list — restoring this rebuilds the per-vehicle tabs (via
    // vehiclesChanged), which now read the restored topic + compute configs.
    const rviz_common::Config vehicles_config = config.mapGetChild( "vehicles" );
    if ( vehicles_config.getType() == rviz_common::Config::List )
    {
        QStringList vehicles;
        for ( int i = 0; i < vehicles_config.listLength(); ++i )
        {
            const QString vehicle = vehicles_config.listChildAt( i ).getValue().toString();
            if ( !vehicle.trimmed().isEmpty() )
            {
                vehicles << vehicle.trimmed();
            }
        }
        setup_component_->setVehicles( vehicles );
    }
}

QWidget* AutonomyPanel::createVehicleTab( QVBoxLayout*& out_content_layout )
{
    // The content widget holds one accordion group per vehicle, stacked vertically.
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
    // Remove existing accordion groups (and any trailing stretch).
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

    // One collapsible accordion group per vehicle, wrapping that vehicle's
    // component flush (no box, no left/right padding).
    for ( const QString& vehicle_id : vehicles )
    {
        AccordionGroup* group = new AccordionGroup( vehicle_id );
        // Tag the group header with the vehicle's label color so it matches the
        // swatch shown for the same vehicle in the Setup table.
        group->setSwatchColor( setup_component_->vehicleColor( vehicle_id ) );
        group->setContentWidget( make_component( vehicle_id ) );
        content_layout->addWidget( group );
    }

    // Keep the accordion groups top-aligned.
    content_layout->addStretch();
}

void AutonomyPanel::onVehiclesChanged( const QStringList& vehicles )
{
    current_vehicles_ = vehicles;

    rebuildVehicleTab( nav_state_layout_, vehicles,
                       [this]( const QString& id ) { return makeNavStateComponent( id ); } );
    rebuildVehicleTab( mission_layout_, vehicles,
                       [this]( const QString& id ) { return makeMissionComponent( id ); } );
    rebuildVehicleTab( tracker_layout_, vehicles,
                       [this]( const QString& id ) { return makeTrackerComponent( id ); } );
    rebuildVehicleTab( compute_layout_, vehicles,
                       [this]( const QString& id ) { return makeComputeComponent( id ); } );
}

void AutonomyPanel::onTopicConfigChanged( TopicGroup group )
{
    // Re-create only the components that consume the changed topic so they
    // re-subscribe with the new name; the rest are left untouched.
    switch ( group )
    {
        case TopicGroup::NavState:
            rebuildVehicleTab( nav_state_layout_, current_vehicles_,
                               [this]( const QString& id ) { return makeNavStateComponent( id ); } );
            break;
        case TopicGroup::Mission:
            rebuildVehicleTab( mission_layout_, current_vehicles_,
                               [this]( const QString& id ) { return makeMissionComponent( id ); } );
            break;
        case TopicGroup::Tracker:
            rebuildVehicleTab( tracker_layout_, current_vehicles_,
                               [this]( const QString& id ) { return makeTrackerComponent( id ); } );
            break;
    }
}

QWidget* AutonomyPanel::makeNavStateComponent( const QString& vehicle_id )
{
    NavStateComponent* component =
        new NavStateComponent( vehicle_id, node_, setup_component_->topicConfig() );

    // Mirror this vehicle's run state into the Setup tab's status table.
    connect( component, &NavStateComponent::navStateChanged, this,
             [this, vehicle_id]( const QString& text, const QColor& color )
             { setup_component_->setVehicleNavState( vehicle_id, text, color ); } );

    return component;
}

QWidget* AutonomyPanel::makeMissionComponent( const QString& vehicle_id )
{
    return new MissionComponent( vehicle_id, node_, setup_component_->topicConfig() );
}

QWidget* AutonomyPanel::makeTrackerComponent( const QString& vehicle_id )
{
    return new TrackerComponent( vehicle_id, node_, setup_component_->topicConfig() );
}

QWidget* AutonomyPanel::makeComputeComponent( const QString& vehicle_id )
{
    ComputeComponent* component = new ComputeComponent( vehicle_id, node_ );

    // Mirror this vehicle's compute health into the Setup tab's status table.
    connect( component, &ComputeComponent::healthChanged, this,
             [this, vehicle_id]( bool healthy )
             { setup_component_->setVehicleComputeHealth( vehicle_id, healthy ); } );

    return component;
}

}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(avt_341::rviz_plugins::AutonomyPanel, rviz_common::Panel )
