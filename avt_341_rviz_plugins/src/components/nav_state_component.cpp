#include <avt_341_rviz_plugins/components/nav_state_component.h>

#include <algorithm>
#include <string>
#include <utility>

#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>

#include <avt_341_msgs/msg/nav_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <avt_341_rviz_plugins/primitives/status_style.h>
#include <avt_341_rviz_plugins/primitives/vector_field.h>
#include <avt_341_rviz_plugins/tf_utils.h>

namespace avt_341::rviz_plugins
{

NavStateComponent::NavStateComponent( const QString& vehicle_id,
                                      rclcpp::Node::SharedPtr node,
                                      const TopicConfig& topics, QWidget* parent )
    : QWidget( parent ), vehicle_id_( vehicle_id ), node_( std::move( node ) ),
      topics_( topics )
{
    buildUi();
    subscribe();
}

void NavStateComponent::buildUi()
{
    // Special axis symbols.
    const QString theta = QString( QChar( 0x03B8 ) );          // theta
    const QString x_hat = QString( "x" ) + QChar( 0x0302 );    // x with combining hat

    // The number-label boxes use VectorField's default light-gray background;
    // no per-axis color overrides are specified.
    // Pose: x, y, theta from odometry.
    pose_field_ = new VectorField(
        "Pose", 3, { "x", "y", theta },
        { "position x", "position y", "yaw (heading)" } );

    // Velocity: linear x/y (odometry) + x-hat desired speed.
    velocity_field_ = new VectorField(
        "Vel", 3, { "x", "y", x_hat },
        { "linear velocity x", "linear velocity y", "desired speed" } );

    // Command: t/s/b -> throttle, steering, brake.
    command_field_ = new VectorField(
        "Cmd", 3, { "t", "s", "b" },
        { "throttle", "steering", "brake" } );

    // Goal: x, y, yaw (theta) from the nav state's goal pose.
    goal_field_ = new VectorField(
        "Goal", 3, { "x", "y", theta },
        { "goal position x", "goal position y", "goal yaw (heading)" } );

    // Nav State row: "<Label>:" + colored status text.
    nav_state_label_ = new QLabel( "State:" );
    nav_state_value_ = new QLabel( "None" );
    nav_state_value_->setAlignment( Qt::AlignCenter );
    nav_state_value_->setStyleSheet( statusBadgeStyleSheet( status_colors::kGray ) );

    QHBoxLayout* nav_state_row = new QHBoxLayout;
    nav_state_row->setContentsMargins( 0, 0, 0, 0 );
    nav_state_row->addWidget( nav_state_label_ );
    // Value stretches to fill the remaining width; its text stays centered.
    nav_state_row->addWidget( nav_state_value_, 1 );

    // Duration row: "<Label>:" + numeric value.
    duration_label_ = new QLabel( "Time:" );
    duration_value_ = new QLabel( "0.00 s" );

    QHBoxLayout* duration_row = new QHBoxLayout;
    duration_row->setContentsMargins( 0, 0, 0, 0 );
    duration_row->addWidget( duration_label_ );
    duration_row->addWidget( duration_value_ );
    duration_row->addStretch();

    // Align every row's label to one shared width so the values line up.
    const int label_width = std::max(
        { pose_field_->labelWidthHint(), velocity_field_->labelWidthHint(),
          command_field_->labelWidthHint(), goal_field_->labelWidthHint(),
          nav_state_label_->sizeHint().width(),
          duration_label_->sizeHint().width() } );
    pose_field_->setLabelWidth( label_width );
    velocity_field_->setLabelWidth( label_width );
    command_field_->setLabelWidth( label_width );
    goal_field_->setLabelWidth( label_width );
    nav_state_label_->setFixedWidth( label_width );
    duration_label_->setFixedWidth( label_width );

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins( 0, 0, 0, 0 );
    layout->addWidget( pose_field_ );
    layout->addWidget( velocity_field_ );
    layout->addWidget( command_field_ );
    // Separate the nav-state section from the basic vehicle state above.
    layout->addSpacing( 12 );
    layout->addLayout( nav_state_row );
    layout->addWidget( goal_field_ );
    layout->addLayout( duration_row );
    layout->addStretch();
    setLayout( layout );
}

void NavStateComponent::subscribe()
{
    // Built before the panel's node exists: render statically, no live values.
    if ( !node_ )
    {
        return;
    }

    // Odometry: best-effort sensor QoS (compatible with reliable or best-effort
    // publishers) for the high-rate pose/twist stream.
    odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        makeTopicPath( vehicle_id_, topics_.odometry ), rclcpp::SensorDataQoS(),
        [this]( nav_msgs::msg::Odometry::ConstSharedPtr msg )
        {
            pose_field_->setValues( { msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      yawOf( msg->pose.pose.orientation ) } );
            linear_velocity_x_ = msg->twist.twist.linear.x;
            linear_velocity_y_ = msg->twist.twist.linear.y;
            updateVelocityField();
        } );

    nav_state_sub_ = node_->create_subscription<avt_341_msgs::msg::NavState>(
        makeTopicPath( vehicle_id_, topics_.nav_state ), rclcpp::QoS( 10 ),
        [this]( avt_341_msgs::msg::NavState::ConstSharedPtr msg )
        {
            setNavStateStatus( msg->run_state );
            goal_field_->setValues( { msg->goal.pose.position.x,
                                      msg->goal.pose.position.y,
                                      yawOf( msg->goal.pose.orientation ) } );
            duration_value_->setText(
                QString::number( msg->goal_duration, 'f', 2 ) + " s" );
        } );

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        makeTopicPath( vehicle_id_, topics_.cmd_vel ), rclcpp::QoS( 10 ),
        [this]( geometry_msgs::msg::Twist::ConstSharedPtr msg )
        {
            // Labels t, s, b -> throttle (linear.x), steering (angular.z),
            // brake (linear.y).
            command_field_->setValues(
                { msg->linear.x, msg->angular.z, msg->linear.y } );
        } );

    desired_speed_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        makeTopicPath( vehicle_id_, topics_.desired_speed ), rclcpp::QoS( 10 ),
        [this]( std_msgs::msg::Float64::ConstSharedPtr msg )
        {
            desired_speed_ = msg->data;
            updateVelocityField();
        } );
}

void NavStateComponent::updateVelocityField()
{
    velocity_field_->setValues(
        { linear_velocity_x_, linear_velocity_y_, desired_speed_ } );
}

void NavStateComponent::setNavStateStatus( int run_state )
{
    QString text;
    QColor color;
    if ( run_state < 0 )
    {
        text = "Startup";
        color = status_colors::kOrange;
    }
    else if ( run_state == 0 )
    {
        text = "Active";
        color = status_colors::kGreen;
    }
    else
    {
        text = "Idle";
        color = status_colors::kGray;
    }
    nav_state_value_->setText( text );
    nav_state_value_->setStyleSheet( statusBadgeStyleSheet( color ) );

    // Mirror the run state to the Setup tab's status table.
    Q_EMIT navStateChanged( text, color );
}

}
