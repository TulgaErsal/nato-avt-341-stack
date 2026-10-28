#include <avt_341_rviz_plugins/components/mission_component.h>

#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>

#include <QAbstractScrollArea>
#include <QBrush>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QList>
#include <QStackedLayout>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <avt_341_rviz_plugins/primitives/icon_utils.h>
#include <avt_341_rviz_plugins/primitives/message_label.h>
#include <avt_341_rviz_plugins/primitives/status_style.h>

namespace
{

// Shown before the first message arrives and for empty string / list fields.
constexpr const char* kEmptyValue = "-";

// The detail rows describe the active task, or -- once the vehicle goes idle --
// the last one that ran. The id row's label says which of the two it is.
constexpr const char* kActiveTaskIdLabel = "Task ID:";
constexpr const char* kLastTaskIdLabel = "Last Task ID:";

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

QString taskSpeedText( double task_speed )
{
    return task_speed < 0.0 ? QString( kEmptyValue )
                            : QString::number( task_speed, 'f', 2 ) + " m/s";
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
    // The task-id row owns its label explicitly because its text is rewritten
    // as the vehicle starts and finishes tasks.
    task_id_label_ = new QLabel( kActiveTaskIdLabel );
    task_id_value_ = new QLabel( kEmptyValue );
    task_description_value_ = new QLabel( kEmptyValue );
    task_speed_value_ = new QLabel( kEmptyValue );
    tracked_vehicle_value_ = new QLabel( kEmptyValue );
    formation_type_value_ = new QLabel( kEmptyValue );
    formation_vehicles_value_ = new QLabel( kEmptyValue );
    formation_offsets_value_ = new QLabel( kEmptyValue );

    // Free-text and the (potentially long) vehicle list wrap instead of forcing
    // the panel wider.
    task_description_value_->setWordWrap( true );
    formation_vehicles_value_->setWordWrap( true );

    // The task list goes in a read-only two-column table (same style as the
    // ComputeComponent status grid): descriptions can be long, so cells give
    // word wrap, per-row heights and scrolling for free. The vertical header
    // stays visible as the 1-based execution position; the second column
    // carries the run time of the tasks that already finished.
    task_list_table_ = new QTableWidget( 0, 2 );
    task_list_table_->setHorizontalHeaderLabels( { "Task", "Duration" } );
    task_list_table_->horizontalHeader()->setSectionsClickable( false );
    task_list_table_->horizontalHeader()->setHighlightSections( false );
    task_list_table_->setEditTriggers( QAbstractItemView::NoEditTriggers );
    task_list_table_->setSelectionMode( QAbstractItemView::NoSelection );
    task_list_table_->setFocusPolicy( Qt::NoFocus );
    task_list_table_->horizontalHeader()->setSectionResizeMode( 0, QHeaderView::Stretch );
    task_list_table_->horizontalHeader()->setSectionResizeMode( 1, QHeaderView::ResizeToContents );
    // Row heights track their (word-wrapped) content.
    task_list_table_->setWordWrap( true );
    task_list_table_->verticalHeader()->setSectionResizeMode( QHeaderView::ResizeToContents );

    // Keep the table tight to its rows; it already sits inside a scroll area.
    task_list_table_->setSizeAdjustPolicy( QAbstractScrollArea::AdjustToContents );

    // Info message centered over the table while it holds no tasks -- the same
    // empty state the Setup tab's vehicle table uses, so the table body stays
    // visible either way.
    MessageLabel* empty_message = new MessageLabel( MessageType::Info, "No tasks." );
    empty_message->setAttribute( Qt::WA_TransparentForMouseEvents );

    task_list_empty_overlay_ = new QWidget();
    task_list_empty_overlay_->setAttribute( Qt::WA_TransparentForMouseEvents );
    QVBoxLayout* empty_layout = new QVBoxLayout( task_list_empty_overlay_ );
    empty_layout->setContentsMargins( 0, 0, 0, 0 );
    empty_layout->addStretch();
    empty_layout->addWidget( empty_message, 0, Qt::AlignHCenter );
    empty_layout->addStretch();

    // Stack the empty-state overlay over the table; the current widget is raised
    // to the front, so the (opaque) table hides the message once it has rows.
    // Nothing has been received yet at construction, so start with the message.
    QWidget* task_list_container = new QWidget();
    task_list_stack_ = new QStackedLayout( task_list_container );
    task_list_stack_->setStackingMode( QStackedLayout::StackAll );
    task_list_stack_->addWidget( task_list_table_ );
    task_list_stack_->addWidget( task_list_empty_overlay_ );
    task_list_stack_->setCurrentWidget( task_list_empty_overlay_ );

    // The table sizes itself to its rows, so without a floor the empty state
    // would collapse to a sliver with no room to read the message in.
    task_list_container->setMinimumHeight( scaledSize( 72, this ) );

    // QFormLayout renders each row as "<Label>: <Value>" with the labels in a
    // shared, right-aligned column so the values line up. No extra margins so the
    // rows are not double-indented inside the surrounding group box.
    QFormLayout* layout = new QFormLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->setFieldGrowthPolicy( QFormLayout::AllNonFixedFieldsGrow );
    layout->addRow( task_id_label_, task_id_value_ );
    layout->addRow( "Task Description:", task_description_value_ );
    layout->addRow( "Task Speed:", task_speed_value_ );
    layout->addRow( "Tracked Vehicle:", tracked_vehicle_value_ );
    layout->addRow( "Formation Type:", formation_type_value_ );
    layout->addRow( "Formation Vehicles:", formation_vehicles_value_ );
    layout->addRow( "Formation Offsets:", formation_offsets_value_ );
    // The task-list label and table each span the full width (no field indent):
    // long descriptions need the label column's width too. The label sits
    // left-aligned on its own row above the table.
    QLabel* task_list_label = new QLabel( "Task List:" );
    task_list_label->setAlignment( Qt::AlignLeft );
    layout->addRow( task_list_label );
    layout->addRow( task_list_container );
    setLayout( layout );

    // Subscribe to this vehicle's task changes. The panel spins the node on the
    // UI thread, so the callback can update these labels directly. Without a
    // node (e.g. built before the panel is initialized) the rows simply stay
    // empty.
    //
    // The module status is only published on task changes, so use a latched
    // (transient-local) QoS matching the publisher: the state published before
    // this component was created is still delivered on join.
    if ( node_ )
    {
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
    // an idle vehicle apart from a vehicle whose publisher went quiet.
    has_active_task_ = hasActiveTask( msg.active_task );

    // The detail rows are only ever written, never blanked: with no active task
    // they keep describing the last one that ran, which the id row's label then
    // says out loud.
    if ( has_active_task_ )
    {
        setActiveTaskFields( msg.active_task );
    }
    task_id_label_->setText( has_active_task_ ? kActiveTaskIdLabel
                                              : kLastTaskIdLabel );

    updateTaskList( msg );
    Q_EMIT taskActiveChanged( has_active_task_ );
}

void MissionComponent::setActiveTaskFields( const avt_341_msgs::msg::MissionTaskStatus& msg )
{
    task_id_value_->setText( taskIdText( msg.task_id ) );
    task_description_value_->setText(
        valueOrDash( QString::fromStdString( msg.task_description ) ) );
    task_speed_value_->setText( taskSpeedText( msg.task_speed ) );
    tracked_vehicle_value_->setText(
        valueOrDash( QString::fromStdString( msg.tracked_vehicle ) ) );
    formation_type_value_->setText(
        valueOrDash( QString::fromStdString( msg.formation_type ) ) );
    formation_offsets_value_->setText(
        msg.tracked_vehicle.empty() ? QString( kEmptyValue )
                                    : QString( "%1, %2" )
                                          .arg( msg.formation_x_offset, 0, 'f', 2 )
                                          .arg( msg.formation_y_offset, 0, 'f', 2 ) );

    // Join the formation vehicles into one comma-separated string.
    QStringList vehicles;
    vehicles.reserve( static_cast<int>( msg.formation_vehicles.size() ) );
    for ( const std::string& vehicle : msg.formation_vehicles )
    {
        vehicles << QString::fromStdString( vehicle );
    }
    formation_vehicles_value_->setText( valueOrDash( vehicles.join( ", " ) ) );
}

void MissionComponent::updateTaskList( const avt_341_msgs::msg::MissionModuleStatus& msg )
{
    // Rows run in execution order -- the tasks that already finished, then the
    // running task, then the queue behind it -- so the row numbers read as the
    // mission's task sequence. Only finished tasks have a run time to show.
    enum class RowKind
    {
        Completed,
        Active,
        Queued
    };
    struct Row
    {
        QString description;
        QString duration;
        RowKind kind;
    };
    QList<Row> rows;

    const int completed_count = static_cast<int>( msg.completed_tasks.size() );
    const int duration_count = static_cast<int>( msg.completed_task_durations.size() );
    for ( int i = 0; i < completed_count; ++i )
    {
        const QString duration =
            i < duration_count
                ? QString::number( msg.completed_task_durations[i], 'f', 1 ) + " s"
                : QString( kEmptyValue );
        rows.append( { valueOrDash( QString::fromStdString( msg.completed_tasks[i] ) ),
                       duration, RowKind::Completed } );
    }
    if ( has_active_task_ )
    {
        rows.append( { valueOrDash( QString::fromStdString( msg.active_task.task_description ) ),
                       QString( kEmptyValue ), RowKind::Active } );
    }
    for ( const std::string& description : msg.queued_tasks )
    {
        rows.append( { valueOrDash( QString::fromStdString( description ) ),
                       QString( kEmptyValue ), RowKind::Queued } );
    }

    const int count = rows.size();
    task_list_stack_->setCurrentWidget( count == 0
                                            ? task_list_empty_overlay_
                                            : static_cast<QWidget*>( task_list_table_ ) );

    task_list_table_->setRowCount( count );
    for ( int row = 0; row < count; ++row )
    {
        QTableWidgetItem* description_item = new QTableWidgetItem( rows.at( row ).description );
        QTableWidgetItem* duration_item = new QTableWidgetItem( rows.at( row ).duration );
        duration_item->setTextAlignment( Qt::AlignRight | Qt::AlignVCenter );
        for ( QTableWidgetItem* item : { description_item, duration_item } )
        {
            // The running task is highlighted; finished tasks are faded out.
            if ( rows.at( row ).kind == RowKind::Active )
            {
                item->setBackground( QBrush( status_colors::kGreen ) );
                item->setForeground( QBrush( Qt::white ) );
            }
            else if ( rows.at( row ).kind == RowKind::Completed )
            {
                item->setForeground( QBrush( status_colors::kGray ) );
            }
        }
        task_list_table_->setItem( row, 0, description_item );
        task_list_table_->setItem( row, 1, duration_item );
    }
}

}
