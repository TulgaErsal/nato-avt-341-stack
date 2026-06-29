#include <avt_341_rviz_plugins/tool_plugins/set_formation_tool.h>

#include <algorithm>
#include <cctype>

#include <QString>
#include <QVariant>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/interaction/view_picker_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace
{

// Vehicle-name fields are published uppercase, matching MissionCommandPanel.
std::string toUpper( std::string s )
{
    std::transform( s.begin(), s.end(), s.begin(),
                    []( unsigned char c ) { return static_cast<char>( std::toupper( c ) ); } );
    return s;
}

}  // namespace

namespace avt_341::rviz_plugins
{

using rviz_common::properties::EnumProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::IntProperty;
using rviz_common::properties::Property;
using rviz_common::properties::StringProperty;

SetFormationTool::SetFormationTool()
{
    shortcut_key_ = 'f';

    command_topic_property_ = new StringProperty(
        "Command Topic", "avt_341/comm_messages",
        "Topic on which the FORM Communication message is published.",
        getPropertyContainer(), SLOT( updateCommandTopic() ), this );

    // --- Marker source / picking -------------------------------------------
    Property* markers_group = new Property(
        "Markers", QVariant(),
        "Where map markers are read from and how clicks are matched to them.",
        getPropertyContainer() );

    marker_topic_property_ = new StringProperty(
        "MapMarker Topic", "avt_341/map_marker",
        "Topic of individual avt_341_msgs/MapMarker messages to match clicks against.",
        markers_group, SLOT( updateMarkerSubscriptions() ), this );

    marker_list_topic_property_ = new StringProperty(
        "MapMarkerList Topic", "avt_341/map_markers_changed",
        "Topic of avt_341_msgs/MapMarkerList messages to match clicks against.",
        markers_group, SLOT( updateMarkerSubscriptions() ), this );

    pick_tolerance_property_ = new FloatProperty(
        "Pick Tolerance", 2.0f,
        "Maximum distance (m) from the clicked point to a marker for the click "
        "to select that marker.",
        markers_group );
    pick_tolerance_property_->setMin( 0.0f );

    // --- FORM Communication fields -----------------------------------------
    Property* form_group = new Property(
        "Formation Message", QVariant(),
        "Fields of the avt_341_msgs/Communication FORM message. msg_id is always "
        "-1, type is always \"FORM\", and objective_name is the clicked marker's id.",
        getPropertyContainer() );

    sender_name_property_ = new StringProperty(
        "Sender Name", "MRZR", "Communication.sender_name (published uppercase).", form_group );
    formation_property_ = new StringProperty(
        "Formation", "", "Communication.formation.", form_group );
    receiver_name_property_ = new StringProperty(
        "Receiver Name", "MRZR", "Communication.receiver_name (published uppercase).", form_group );
    leader_name_property_ = new StringProperty(
        "Leader Name", "MRZR", "Communication.leader_name (published uppercase).", form_group );
    follower1_property_ = new StringProperty(
        "Follower 1 Name", "", "Communication.follower1_name (published uppercase).", form_group );
    follower2_property_ = new StringProperty(
        "Follower 2 Name", "", "Communication.follower2_name (published uppercase).", form_group );
    follower3_property_ = new StringProperty(
        "Follower 3 Name", "", "Communication.follower3_name (published uppercase).", form_group );

    desired_speed_property_ = new FloatProperty(
        "Desired Speed", 5.0f, "Communication.desired_speed (m/s).", form_group );

    priority_type_property_ = new EnumProperty(
        "Priority Type", "QUEUE", "Communication.priority_type.", form_group );
    priority_type_property_->addOption( "QUEUE", 0 );
    priority_type_property_->addOption( "PREEMPT", 1 );
    priority_type_property_->addOption( "CANCEL_ALL", 2 );

    termination_method_property_ = new EnumProperty(
        "Termination Method", "LEADER_ARRIVED", "Communication.termination_method.", form_group );
    termination_method_property_->addOption( "LEADER_ARRIVED", 0 );
    termination_method_property_->addOption( "ALL_ARRIVED", 1 );

    x_scale_property_ = new FloatProperty( "X Scale", 1.0f, "Communication.x_scale.", form_group );
    y_scale_property_ = new FloatProperty( "Y Scale", 1.0f, "Communication.y_scale.", form_group );
    x_offset_property_ = new FloatProperty( "X Offset", 0.0f, "Communication.x_offset.", form_group );
    y_offset_property_ = new FloatProperty( "Y Offset", 0.0f, "Communication.y_offset.", form_group );
    distance_property_ = new FloatProperty( "Distance", 0.0f, "Communication.distance.", form_group );
    target_msg_id_property_ = new IntProperty(
        "Target Message ID", 0, "Communication.target_msg_id.", form_group );
}

SetFormationTool::~SetFormationTool() = default;

void SetFormationTool::onInitialize()
{
    setName( "Set Formation" );
    node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    updateCommandTopic();
    updateMarkerSubscriptions();
}

void SetFormationTool::activate()
{
    setStatus( "Click a map marker to publish a FORM formation with it as the objective." );
}

void SetFormationTool::deactivate()
{
}

void SetFormationTool::updateCommandTopic()
{
    if ( !node_ )
    {
        return;
    }
    publisher_ = node_->create_publisher<avt_341_msgs::msg::Communication>(
        command_topic_property_->getStdString(), rclcpp::QoS( 10 ) );
}

void SetFormationTool::updateMarkerSubscriptions()
{
    if ( !node_ )
    {
        return;
    }

    // Latched QoS to match the transient-local marker publishers, so the most
    // recently published markers arrive immediately on (re)subscribe.
    const rclcpp::QoS qos = rclcpp::QoS( rclcpp::KeepLast( 5 ) ).reliable().transient_local();

    {
        std::lock_guard<std::mutex> lock( markers_mutex_ );
        single_markers_.clear();
        list_markers_.clear();
    }

    marker_sub_ = node_->create_subscription<avt_341_msgs::msg::MapMarker>(
        marker_topic_property_->getStdString(), qos,
        [this]( avt_341_msgs::msg::MapMarker::ConstSharedPtr msg ) { onMarker( msg ); } );

    marker_list_sub_ = node_->create_subscription<avt_341_msgs::msg::MapMarkerList>(
        marker_list_topic_property_->getStdString(), qos,
        [this]( avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg ) { onMarkerList( msg ); } );
}

void SetFormationTool::onMarker( avt_341_msgs::msg::MapMarker::ConstSharedPtr msg )
{
    CachedMarker marker;
    marker.marker_id = msg->marker_id;
    marker.frame_id = msg->header.frame_id;
    marker.x = msg->pose.position.x;
    marker.y = msg->pose.position.y;
    marker.z = msg->pose.position.z;

    std::lock_guard<std::mutex> lock( markers_mutex_ );
    single_markers_.assign( 1, marker );  // the single-marker topic holds the latest one
}

void SetFormationTool::onMarkerList( avt_341_msgs::msg::MapMarkerList::ConstSharedPtr msg )
{
    std::vector<CachedMarker> markers;
    markers.reserve( msg->markers.size() );
    for ( const avt_341_msgs::msg::MapMarker& in : msg->markers )
    {
        CachedMarker marker;
        marker.marker_id = in.marker_id;
        // Each marker may carry its own frame; fall back to the list's frame.
        marker.frame_id = in.header.frame_id.empty() ? msg->header.frame_id : in.header.frame_id;
        marker.x = in.pose.position.x;
        marker.y = in.pose.position.y;
        marker.z = in.pose.position.z;
        markers.push_back( marker );
    }

    std::lock_guard<std::mutex> lock( markers_mutex_ );
    list_markers_ = std::move( markers );
}

bool SetFormationTool::findNearestMarker( double px, double py,
                                          std::string& marker_id_out ) const
{
    rviz_common::FrameManagerIface* frame_manager = context_->getFrameManager();
    const double tolerance = pick_tolerance_property_->getFloat();

    double best_dist_sq = tolerance * tolerance;
    bool found = false;

    std::lock_guard<std::mutex> lock( markers_mutex_ );

    const auto consider = [&]( const std::vector<CachedMarker>& markers )
    {
        for ( const CachedMarker& marker : markers )
        {
            Ogre::Vector3 frame_pos;
            Ogre::Quaternion frame_ori;
            if ( !frame_manager->getTransform( marker.frame_id, frame_pos, frame_ori ) )
            {
                continue;  // marker's frame is not currently known to TF
            }

            const Ogre::Vector3 world =
                frame_pos + frame_ori * Ogre::Vector3( marker.x, marker.y, marker.z );

            // Compare in the ground plane; markers are flat on the ground while
            // the picked point may sit slightly above it.
            const double dx = world.x - px;
            const double dy = world.y - py;
            const double dist_sq = dx * dx + dy * dy;
            if ( dist_sq <= best_dist_sq )
            {
                best_dist_sq = dist_sq;
                marker_id_out = marker.marker_id;
                found = true;
            }
        }
    };

    consider( single_markers_ );
    consider( list_markers_ );
    return found;
}

void SetFormationTool::publishFormation( const std::string& objective_name )
{
    if ( !publisher_ )
    {
        return;
    }

    avt_341_msgs::msg::Communication msg;
    msg.sender_name = toUpper( sender_name_property_->getStdString() );
    msg.msg_id = -1;
    msg.type = "FORM";
    msg.formation = formation_property_->getStdString();
    msg.receiver_name = toUpper( receiver_name_property_->getStdString() );
    msg.leader_name = toUpper( leader_name_property_->getStdString() );
    msg.follower1_name = toUpper( follower1_property_->getStdString() );
    msg.follower2_name = toUpper( follower2_property_->getStdString() );
    msg.follower3_name = toUpper( follower3_property_->getStdString() );
    msg.objective_name = objective_name;
    msg.desired_speed = desired_speed_property_->getFloat();
    msg.priority_type = priority_type_property_->getStdString();
    msg.termination_method = termination_method_property_->getStdString();
    msg.x_scale = x_scale_property_->getFloat();
    msg.y_scale = y_scale_property_->getFloat();
    msg.x_offset = x_offset_property_->getFloat();
    msg.y_offset = y_offset_property_->getFloat();
    msg.distance = distance_property_->getFloat();
    msg.target_msg_id = target_msg_id_property_->getInt();
    // yaw_threshold is left at its message default (-1), as MissionCommandPanel does.

    publisher_->publish( msg );
}

int SetFormationTool::processMouseEvent( rviz_common::ViewportMouseEvent& event )
{
    if ( !event.leftUp() )
    {
        return 0;
    }

    Ogre::Vector3 point;
    if ( !context_->getViewPicker()->get3DPoint( event.panel, event.x, event.y, point ) )
    {
        setStatus( "No object under the cursor - click directly on a map marker." );
        return Render;
    }

    std::string marker_id;
    if ( !findNearestMarker( point.x, point.y, marker_id ) )
    {
        setStatus( "No map marker within the pick tolerance of the click." );
        return Render;
    }

    publishFormation( marker_id );
    setStatus( QString( "Published FORM formation for marker '%1'." )
                   .arg( QString::fromStdString( marker_id ) ) );
    return Render;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( avt_341::rviz_plugins::SetFormationTool, rviz_common::Tool )
