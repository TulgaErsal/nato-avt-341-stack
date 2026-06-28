#include <avt_341_rviz_plugins/components/tracker_component.h>

#include <algorithm>
#include <string>
#include <utility>

#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>

#include <avt_341_msgs/msg/tracker_info.hpp>

#include <avt_341_rviz_plugins/primitives/matrix_field.h>
#include <avt_341_rviz_plugins/primitives/vector_field.h>

namespace
{

// Status row colors (shared palette with the other components).
const QColor kGray( 108, 117, 125 );   // uninitialized / inactive / unknown
const QColor kOrange( 230, 126, 34 );  // no detection / camera-only tracking
const QColor kGreen( 40, 167, 69 );    // full / lidar-only tracking
const QColor kRed( 220, 53, 69 );      // lost

// Stylesheet for the state value label given a background color.
QString statusStyleSheet( const QColor& color )
{
    return QString( "background-color: %1; color: white; padding: 2px 8px; "
                    "border-radius: 2px;" ).arg( color.name() );
}

}  // namespace

namespace avt_341::rviz_plugins
{

using avt_341_msgs::msg::TrackerInfo;

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
    // State row: "<Label>:" + colored status text. Shows "None" until the first
    // TrackerInfo message arrives.
    state_label_ = new QLabel( "State:" );
    state_value_ = new QLabel( "None" );
    state_value_->setAlignment( Qt::AlignCenter );
    state_value_->setStyleSheet( statusStyleSheet( kGray ) );

    QHBoxLayout* state_row = new QHBoxLayout;
    state_row->setContentsMargins( 0, 0, 0, 0 );
    state_row->addWidget( state_label_ );
    // Value stretches to fill the remaining width; its text stays centered.
    state_row->addWidget( state_value_, 1 );

    // Covariance row: a labelled 3x3 matrix of the tracker estimate's covariance.
    // Cells default to 0 (placeholder until the tracker message carries them).
    covariance_field_ = new MatrixField( "Covariance", 1.0, 10.0 );

    // Target row: the tracked target's range (d) and bearing (theta) as a
    // two-element vector. Fields default to 0.00 (placeholder for now).
    const QString theta = QString( QChar( 0x03B8 ) );  // theta symbol
    target_field_ = new VectorField(
        "Target", 2, { "d", theta }, { "distance", "bearing (theta)" } );

    // Align every row's label to one shared width so the values line up across
    // the State, Covariance and Target rows (same approach as NavStateComponent).
    const int label_width = std::max(
        { state_label_->sizeHint().width(), covariance_field_->labelWidthHint(),
          target_field_->labelWidthHint() } );
    state_label_->setFixedWidth( label_width );
    covariance_field_->setLabelWidth( label_width );
    target_field_->setLabelWidth( label_width );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addLayout( state_row );
    layout->addWidget( covariance_field_ );
    layout->addWidget( target_field_ );
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

    const std::string topic =
        "/" + vehicle_id_.toStdString() + "/" + topics_.tracker_state.toStdString();

    tracker_sub_ = node_->create_subscription<TrackerInfo>(
        topic, rclcpp::QoS( 10 ),
        [this]( TrackerInfo::ConstSharedPtr msg ) { setTrackerStatus( msg->state ); } );
}

void TrackerComponent::setTrackerStatus( uint8_t state )
{
    QString text;
    QColor color;
    switch ( state )
    {
        case TrackerInfo::STATE_UNINITIALIZED:
            text = "Uninitialized";
            color = kGray;
            break;
        case TrackerInfo::STATE_INACTIVE:
            text = "Inactive";
            color = kGray;
            break;
        case TrackerInfo::STATE_NO_DETECTION:
            text = "No Detection";
            color = kOrange;
            break;
        case TrackerInfo::STATE_LIDAR_ONLY_TRACKING:
            text = "LiDAR Only Tracking";
            color = kGreen;
            break;
        case TrackerInfo::STATE_FULL_TRACKING:
            text = "Full Tracking";
            color = kGreen;
            break;
        case TrackerInfo::STATE_CAMERA_ONLY_TRACKING:
            text = "Camera Only Tracking";
            color = kOrange;
            break;
        case TrackerInfo::STATE_LOST:
            text = "Lost";
            color = kRed;
            break;
        default:
            text = QString( "Unknown (%1)" ).arg( static_cast<int>( state ) );
            color = kGray;
            break;
    }
    state_value_->setText( text );
    state_value_->setStyleSheet( statusStyleSheet( color ) );
}

}
