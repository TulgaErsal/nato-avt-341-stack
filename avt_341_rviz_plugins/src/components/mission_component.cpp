#include <avt_341_rviz_plugins/components/mission_component.h>

#include <cstdint>
#include <string>
#include <utility>

#include <QAbstractScrollArea>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>

namespace
{

// Shown before the first message arrives and for empty string / list fields.
constexpr const char* kEmptyValue = "-";

QString valueOrDash( const QString& value )
{
    return value.isEmpty() ? QString( kEmptyValue ) : value;
}

// True when the message describes a task that is actually running.
//
// The id alone is not a sufficient test. MissionTaskStatus documents
// task_id = -1 as "no task" and the mission manager's empty status uses it, but
// the tasks the manager creates for itself -- contact investigation, overwatch,
// wait-for-completion -- are also built with msg_id = -1. Those carry a real
// description, so a non-empty description still means a live task.
bool hasActiveTask( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    return msg.task_id >= 0 || !msg.task_description.empty();
}

// Task ids are only meaningful when non-negative; see hasActiveTask.
QString taskIdText( std::int32_t task_id )
{
    return task_id < 0 ? QString( kEmptyValue ) : QString::number( task_id );
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

    // Queued tasks go in a read-only single-column table (same style as the
    // ComputeComponent status grid): descriptions can be long, so cells give
    // word wrap, per-row heights and scrolling for free. The vertical header
    // stays visible as the 1-based queue position; the column header is hidden
    // because the "Queued Tasks:" label above the table already titles it.
    queued_tasks_table_ = new QTableWidget( 0, 1 );
    queued_tasks_table_->horizontalHeader()->setVisible( false );
    queued_tasks_table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    queued_tasks_table_->setSelectionMode( QAbstractItemView::NoSelection );
    queued_tasks_table_->setFocusPolicy( Qt::NoFocus );
    queued_tasks_table_->horizontalHeader()->setSectionResizeMode( 0, QHeaderView::Stretch );
    // Row heights track their (word-wrapped) content.
    queued_tasks_table_->setWordWrap( true );
    queued_tasks_table_->verticalHeader()->setSectionResizeMode( QHeaderView::ResizeToContents );

    // Keep the table tight to its rows; it already sits inside a scroll area.
    queued_tasks_table_->setSizeAdjustPolicy( QAbstractScrollArea::AdjustToContents );

    // Stand-in shown instead of the table while the queue is empty (same
    // approach as TrackerComponent's "No active trackers."). Nothing has been
    // received yet at construction, so start in the empty state.
    queued_tasks_empty_label_ = new QLabel( "No queued tasks." );
    queued_tasks_table_->setVisible( false );

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
    // The queued-tasks label and table each span the full width (no field
    // indent): long descriptions need the label column's width too. The label
    // sits left-aligned on its own row above the table.
    QLabel* queued_tasks_label = new QLabel( "Queued Tasks:" );
    queued_tasks_label->setAlignment( Qt::AlignLeft );
    layout->addRow( queued_tasks_label );
    layout->addRow( queued_tasks_empty_label_ );
    layout->addRow( queued_tasks_table_ );
    setLayout( layout );

    // Subscribe to this vehicle's task status. The panel spins the node on the UI
    // thread, so the callback can update these labels directly. Without a node
    // (e.g. built before the panel is initialized) the rows simply stay empty.
    if ( node_ )
    {
        const std::string topic = makeTopicPath( vehicle_id_, topics_.task_status );
        subscription_ = node_->create_subscription<avt_341_msgs::msg::MissionTaskStatus>(
            topic, rclcpp::QoS( 10 ),
            [this]( avt_341_msgs::msg::MissionTaskStatus::ConstSharedPtr msg )
            {
                updateFromMessage( *msg );
            } );

        // The module status is only published on task changes, so use a latched
        // (transient-local) QoS matching the publisher: the state published
        // before this component was created is still delivered on join.
        const rclcpp::QoS latched_qos =
            rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();
        const std::string task_change_topic =
            makeTopicPath( vehicle_id_, topics_.task_change );
        task_change_subscription_ =
            node_->create_subscription<avt_341_msgs::msg::MissionModuleStatus>(
                task_change_topic, latched_qos,
                [this]( avt_341_msgs::msg::MissionModuleStatus::ConstSharedPtr msg )
                {
                    updateFromModuleStatus( *msg );
                } );
    }
}

void MissionComponent::updateFromModuleStatus( const avt_341_msgs::msg::MissionModuleStatus& msg )
{
    // This topic is the authority: it is published on every task-list change,
    // including the one that empties it, and so is the only thing that can tell
    // an idle vehicle apart from a vehicle whose status stream went quiet.
    has_active_task_ = hasActiveTask( msg.active_task );
    active_task_id_ = msg.active_task.task_id;

    if ( has_active_task_ )
    {
        setActiveTaskFields( msg.active_task );
    }
    else
    {
        clearActiveTask();
    }

    updateQueuedTasks( msg );
}

void MissionComponent::updateFromMessage( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    // The mission manager publishes nothing here while no task is running, so a
    // message arriving is not evidence that one is. Apply it only when it
    // refreshes the task the module status named; anything else is stale or
    // belongs to a task that has already been popped.
    if ( !has_active_task_ || msg.task_id != active_task_id_ )
    {
        return;
    }
    setActiveTaskFields( msg );
}

void MissionComponent::setActiveTaskFields( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    task_id_value_->setText( taskIdText( msg.task_id ) );
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

void MissionComponent::clearActiveTask()
{
    task_id_value_->setText( kEmptyValue );
    task_description_value_->setText( kEmptyValue );
    tracked_vehicle_value_->setText( kEmptyValue );
    formation_type_value_->setText( kEmptyValue );
    formation_vehicles_value_->setText( kEmptyValue );
}

void MissionComponent::updateQueuedTasks( const avt_341_msgs::msg::MissionModuleStatus& msg )
{
    const int count = static_cast<int>( msg.queued_tasks.size() );
    queued_tasks_empty_label_->setVisible( count == 0 );
    queued_tasks_table_->setVisible( count > 0 );

    queued_tasks_table_->setRowCount( count );
    for ( int row = 0; row < count; ++row )
    {
        queued_tasks_table_->setItem(
            row, 0,
            new QTableWidgetItem( QString::fromStdString( msg.queued_tasks[row] ) ) );
    }
}

}
