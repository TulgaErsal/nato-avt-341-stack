#include <avt_341_rviz_plugins/components/tracker_component.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include <QChar>
#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <QVector>

#include <avt_341_msgs/msg/tracker_status.hpp>

#include <avt_341_rviz_plugins/primitives/accordion_group.h>
#include <avt_341_rviz_plugins/primitives/matrix_field.h>
#include <avt_341_rviz_plugins/primitives/status_style.h>
#include <avt_341_rviz_plugins/primitives/vector_field.h>

namespace avt_341::rviz_plugins
{

namespace
{

using avt_341_msgs::msg::TrackerStatus;

// Sets a state value label's text + background from a TrackerStatus.state.
void applyTrackerState( QLabel* value, std::uint8_t state )
{
    QString text;
    QColor color;
    switch ( state )
    {
        case TrackerStatus::STATE_UNINITIALIZED:
            text = "Uninitialized";
            color = status_colors::kGray;
            break;
        case TrackerStatus::STATE_INACTIVE:
            text = "Inactive";
            color = status_colors::kGray;
            break;
        case TrackerStatus::STATE_NO_DETECTION:
            text = "No Detection";
            color = status_colors::kOrange;
            break;
        case TrackerStatus::STATE_LIDAR_ONLY_TRACKING:
            text = "LiDAR Only Tracking";
            color = status_colors::kGreen;
            break;
        case TrackerStatus::STATE_FULL_TRACKING:
            text = "Full Tracking";
            color = status_colors::kGreen;
            break;
        case TrackerStatus::STATE_CAMERA_ONLY_TRACKING:
            text = "Camera Only Tracking";
            color = status_colors::kOrange;
            break;
        case TrackerStatus::STATE_LOST:
            text = "Lost";
            color = status_colors::kRed;
            break;
        default:
            text = QString( "Unknown (%1)" ).arg( static_cast<int>( state ) );
            color = status_colors::kGray;
            break;
    }
    value->setText( text );
    value->setStyleSheet( statusBadgeStyleSheet( color ) );
}

// Pulls the x/y/yaw 3x3 sub-block (row-major) out of a 6x6 pose covariance
// laid out as [x, y, z, roll, pitch, yaw] -> indices 0, 1, 5.
QVector<double> extractXYYawCovariance( const std::array<double, 36>& covariance )
{
    const int idx[3] = { 0, 1, 5 };
    QVector<double> out;
    out.reserve( 9 );
    for ( int r = 0; r < 3; ++r )
    {
        for ( int c = 0; c < 3; ++c )
        {
            out.append( covariance[idx[r] * 6 + idx[c]] );
        }
    }
    return out;
}

}  // namespace

using avt_341_msgs::msg::TrackerModuleStatus;

TrackerComponent::TrackerComponent( const QString& vehicle_id,
                                    rclcpp::Node::SharedPtr node,
                                    const TopicConfig& topics, QWidget* parent )
    : QWidget( parent ), vehicle_id_( vehicle_id ), node_( std::move( node ) ),
      topics_( topics )
{
    buildUi();
    subscribe();
}

void TrackerComponent::buildUi()
{
    // Shown while the tracker module has no active child trackers.
    empty_label_ = new QLabel( "No active trackers." );

    // Holds one collapsible "Tracked <id>" sub-group per target, slightly inset
    // from the vehicle group's left edge.
    targets_layout_ = new QVBoxLayout;
    targets_layout_->setContentsMargins( 4, 0, 0, 0 );
    targets_layout_->setSpacing( 6 );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( empty_label_ );
    layout->addLayout( targets_layout_ );
    layout->addStretch();
    setLayout( layout );
}

void TrackerComponent::subscribe()
{
    // Built before the panel's node exists: render statically, no live values.
    if ( !node_ )
    {
        return;
    }

    const std::string topic = makeTopicPath( vehicle_id_, topics_.tracker_state );

    tracker_sub_ = node_->create_subscription<TrackerModuleStatus>(
        topic, rclcpp::QoS( 10 ),
        [this]( TrackerModuleStatus::ConstSharedPtr msg ) { updateFromMessage( *msg ); } );
}

void TrackerComponent::updateFromMessage( const TrackerModuleStatus& msg )
{
    // Build the id list for this message and rebuild the sub-groups only if the
    // set/order of targets changed, so steady-state updates preserve each
    // group's expand/collapse state.
    QStringList ids;
    ids.reserve( static_cast<int>( msg.trackers.size() ) );
    for ( const auto& tracker : msg.trackers )
    {
        ids << QString::fromStdString( tracker.tracked_object_id );
    }

    QStringList current;
    current.reserve( static_cast<int>( target_views_.size() ) );
    for ( const TargetView& view : target_views_ )
    {
        current << view.object_id;
    }
    if ( ids != current )
    {
        rebuildTargets( ids );
    }

    empty_label_->setVisible( msg.trackers.empty() );

    const std::size_t n = std::min( msg.trackers.size(), target_views_.size() );
    for ( std::size_t i = 0; i < n; ++i )
    {
        const auto& tracker = msg.trackers[i];
        TargetView& view = target_views_[i];

        applyTrackerState( view.state_value, tracker.state );
        view.covariance_field->setValues(
            extractXYYawCovariance( tracker.odom_estimate.pose.covariance ) );

        // Target range/bearing derived from the estimated position.
        const auto& position = tracker.odom_estimate.pose.pose.position;
        const double range = std::hypot( position.x, position.y );
        const double bearing = std::atan2( position.y, position.x );
        view.target_field->setValues( { range, bearing } );
    }
}

void TrackerComponent::rebuildTargets( const QStringList& ids )
{
    // Drop the existing sub-groups (deleting a group deletes its content).
    for ( TargetView& view : target_views_ )
    {
        targets_layout_->removeWidget( view.group );
        view.group->deleteLater();
    }
    target_views_.clear();

    const QString theta = QString( QChar( 0x03B8 ) );  // theta symbol

    for ( const QString& id : ids )
    {
        TargetView view;
        view.object_id = id;

        // State row: "<Label>:" + colored status text.
        QLabel* state_label = new QLabel( "State:" );
        view.state_value = new QLabel( "None" );
        view.state_value->setAlignment( Qt::AlignCenter );
        view.state_value->setStyleSheet( statusBadgeStyleSheet( status_colors::kGray ) );

        QHBoxLayout* state_row = new QHBoxLayout;
        state_row->setContentsMargins( 0, 0, 0, 0 );
        state_row->addWidget( state_label );
        state_row->addWidget( view.state_value, 1 );

        // Covariance: the x/y/yaw 3x3 sub-matrix of the estimate's pose covariance.
        view.covariance_field = new MatrixField( "Covariance", 1.0, 10.0 );

        // Target: range (d) + bearing (theta).
        view.target_field = new VectorField(
            "Target", 2, { "d", theta }, { "distance", "bearing (theta)" } );

        // Shared label width so the three rows line up (as in NavStateComponent).
        const int label_width = std::max(
            { state_label->sizeHint().width(), view.covariance_field->labelWidthHint(),
              view.target_field->labelWidthHint() } );
        state_label->setFixedWidth( label_width );
        view.covariance_field->setLabelWidth( label_width );
        view.target_field->setLabelWidth( label_width );

        QWidget* content = new QWidget;
        QVBoxLayout* content_layout = new QVBoxLayout( content );
        content_layout->setContentsMargins( 0, 0, 0, 0 );
        content_layout->addLayout( state_row );
        content_layout->addWidget( view.covariance_field );
        content_layout->addWidget( view.target_field );

        // Sub-group header "Tracked <id>"; no color swatch on target sub-groups
        // (AccordionGroup hides the swatch unless setSwatchColor() is called).
        view.group = new AccordionGroup( "Tracked " + id );
        view.group->setContentWidget( content );

        targets_layout_->addWidget( view.group );
        target_views_.push_back( view );
    }
}

}
