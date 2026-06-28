#include <avt_341_rviz_plugins/components/mission_component.h>

#include <string>
#include <utility>

#include <QFormLayout>
#include <QLabel>
#include <QStringList>

namespace
{

// Shown before the first message arrives and for empty string / list fields.
constexpr const char* kEmptyValue = "-";

QString valueOrDash( const QString& value )
{
    return value.isEmpty() ? QString( kEmptyValue ) : value;
}

}  // namespace

namespace avt_341::rviz_plugins
{

MissionComponent::MissionComponent( const QString& vehicle_id,
                                    rclcpp::Node::SharedPtr node,
                                    const TopicConfig& topics, QWidget* parent )
    : QWidget( parent ), vehicle_id_( vehicle_id ), node_( std::move( node ) ),
      topics_( topics )
{
    // One value label per shown field; the labels carry the "<Label>:" text.
    task_id_value_ = new QLabel( kEmptyValue );
    task_description_value_ = new QLabel( kEmptyValue );
    tracked_vehicle_value_ = new QLabel( kEmptyValue );
    formation_type_value_ = new QLabel( kEmptyValue );
    formation_vehicles_value_ = new QLabel( kEmptyValue );

    // Free-text and the (potentially long) vehicle list wrap instead of forcing
    // the panel wider.
    task_description_value_->setWordWrap( true );
    formation_vehicles_value_->setWordWrap( true );

    // QFormLayout renders each row as "<Label>: <Value>" with the labels in a
    // shared, right-aligned column so the values line up. No extra margins so the
    // rows are not double-indented inside the surrounding group box.
    QFormLayout* layout = new QFormLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->setFieldGrowthPolicy( QFormLayout::AllNonFixedFieldsGrow );
    layout->addRow( "Task ID:", task_id_value_ );
    layout->addRow( "Task Description:", task_description_value_ );
    layout->addRow( "Tracked Vehicle:", tracked_vehicle_value_ );
    layout->addRow( "Formation Type:", formation_type_value_ );
    layout->addRow( "Formation Vehicles:", formation_vehicles_value_ );
    setLayout( layout );

    // Subscribe to this vehicle's task status. The panel spins the node on the UI
    // thread, so the callback can update these labels directly. Without a node
    // (e.g. built before the panel is initialized) the rows simply stay empty.
    if ( node_ )
    {
        const std::string topic =
            ( "/" + vehicle_id_ + "/" + topics_.task_status ).toStdString();
        subscription_ = node_->create_subscription<avt_341_msgs::msg::MissionTaskStatus>(
            topic, rclcpp::QoS( 10 ),
            [this]( avt_341_msgs::msg::MissionTaskStatus::ConstSharedPtr msg )
            {
                updateFromMessage( *msg );
            } );
    }
}

void MissionComponent::updateFromMessage( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    task_id_value_->setText( QString::number( msg.task_id ) );
    task_description_value_->setText(
        valueOrDash( QString::fromStdString( msg.task_description ) ) );
    tracked_vehicle_value_->setText(
        valueOrDash( QString::fromStdString( msg.tracked_vehicle ) ) );
    formation_type_value_->setText(
        valueOrDash( QString::fromStdString( msg.formation_type ) ) );

    // Join the formation vehicles into one comma-separated string.
    QStringList vehicles;
    vehicles.reserve( static_cast<int>( msg.formation_vehicles.size() ) );
    for ( const std::string& vehicle : msg.formation_vehicles )
    {
        vehicles << QString::fromStdString( vehicle );
    }
    formation_vehicles_value_->setText( valueOrDash( vehicles.join( ", " ) ) );
}

}
