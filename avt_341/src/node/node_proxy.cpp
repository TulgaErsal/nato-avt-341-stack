#include <avt_341/node/node_proxy.h>

#ifdef ROS_1

#else
  #ifdef GTE_ROS_HUMBLE
  #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
  #else
  #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
  #endif
#endif

namespace avt_341 {
namespace node {

#ifdef ROS_1

Rate::Rate(double hz) : rate_(hz) {
}

void Rate::sleep() {
    rate_.sleep();
}

NodeProxy::NodeProxy(const std::string &node_name) {
}

void NodeProxy::initialize_tf_listener() {
  if(tf_buffer_ != nullptr)
    return;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, ros::Time(0));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
  
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame, const ros::Time &stamp){
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, stamp, ros::Duration(0.2));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
}

geometry_msgs::TransformStamped NodeProxy::lookup_transform(const std::string& target_frame, const ros::Time& target_time,
                                                            const std::string& source_frame, const ros::Time& source_time,
                                                            const std::string& fixed_frame){
  try {
    return tf_buffer_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration(0.2));
  } catch (const tf2::TransformException & ex) {
    //n->log_warning("Could not transform %s to %s: %s", frame_world.c_str(), frame_cg.c_str(), ex.what());
    return geometry_msgs::TransformStamped();
  }
}

bool NodeProxy::transform_cloud(const sensor_msgs::PointCloud2 & in_cloud, sensor_msgs::PointCloud2 & out_cloud, const std::string &target_frame, bool deskew_lidar){
	try {
		tf_buffer_->transform(in_cloud, out_cloud, target_frame, ros::Duration(0.2));
	} catch (const tf2::TransformException & ex) {
		log_warning("Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
		return false;
	}
   return true;
}

bool NodeProxy::transform_cloud(const sensor_msgs::PointCloud2 & in_cloud, sensor_msgs::PointCloud2 & out_cloud,
                                const std::string &target_frame, const ros::Time& target_time, const std::string &fixed_frame) {
	try {
		tf_buffer_->transform(in_cloud, out_cloud, target_frame, target_time, fixed_frame, ros::Duration(0.2));
	} catch (const tf2::TransformException & ex) {
		log_warning("Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
		return false;
	}
   return true;
}

bool NodeProxy::transform_pose(const geometry_msgs::PoseStamped & in_pose, geometry_msgs::PoseStamped & out_pose, const std::string &target_frame, float duration) {
	try {
		tf_buffer_->transform(in_pose, out_pose, target_frame, ros::Duration(duration));
	} catch (const tf2::TransformException & ex) {
		log_warning("Could not transform pose %s to %s: %s", out_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
		return false;
	}
   return true;
}

void NodeProxy::publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.frame_id = parent_frame;
      tf_msg.child_frame_id = child_frame;

      tf_msg.transform.translation.x = target_pose.pose.position.x;
      tf_msg.transform.translation.y = target_pose.pose.position.y;
      tf_msg.transform.translation.z = target_pose.pose.position.z;
      tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
      tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
      tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
      tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

      tf_broadcaster_->sendTransform(tf_msg);
}

void NodeProxy::publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::PoseStamped &target_pose) {
  if(tf_buffer_ == nullptr) {
    initialize_tf_listener();
  }

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = target_pose.pose.position.x;
  tf_msg.transform.translation.y = target_pose.pose.position.y;
  tf_msg.transform.translation.z = target_pose.pose.position.z;
  tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
  tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
  tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
  tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

  tf_static_broadcaster_->sendTransform(tf_msg);
}

double NodeProxy::get_now_seconds() const {
    return get_stamp().toSec();
}

ros::Time NodeProxy::get_stamp() const {
    return ros::Time::now();
}

void NodeProxy::spin_some() {
    ros::spinOnce();
}

void NodeProxy::spin() {
    ros::spin();
}

#else

    Rate::Rate(double hz) : rate_(hz) {
    }

    void Rate::sleep() {
      rate_.sleep();
    }

    NodeProxy::NodeProxy(const std::string &node_name) {
      node_ = rclcpp::Node::make_shared(node_name);
      this->get_parameter("/is_empty_waypoints", is_empty_waypoints_, false);
    }

    void NodeProxy::initialize_tf_listener() {
      if(tf_buffer_ != nullptr)
        return;

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
      tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    }

    void NodeProxy::publish_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = parent_frame;
      tf_msg.child_frame_id = child_frame;

      tf_msg.transform.translation.x = target_pose.pose.position.x;
      tf_msg.transform.translation.y = target_pose.pose.position.y;
      tf_msg.transform.translation.z = target_pose.pose.position.z;
      tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
      tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
      tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
      tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

      tf_broadcaster_->sendTransform(tf_msg);
    }

    void NodeProxy::publish_static_tf(const std::string &parent_frame, const std::string &child_frame, const geometry_msgs::msg::PoseStamped &target_pose) {
      if(tf_buffer_ == nullptr) {
        initialize_tf_listener();
      }

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.frame_id = parent_frame;
      tf_msg.child_frame_id = child_frame;

      tf_msg.transform.translation.x = target_pose.pose.position.x;
      tf_msg.transform.translation.y = target_pose.pose.position.y;
      tf_msg.transform.translation.z = target_pose.pose.position.z;
      tf_msg.transform.rotation.x = target_pose.pose.orientation.x;
      tf_msg.transform.rotation.y = target_pose.pose.orientation.y;
      tf_msg.transform.rotation.z = target_pose.pose.orientation.z;
      tf_msg.transform.rotation.w = target_pose.pose.orientation.w;

      tf_static_broadcaster_->sendTransform(tf_msg);
    }


    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame){
      try {
        return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time & stamp){
      try {
        return tf_buffer_->lookupTransform(target_frame, source_frame, stamp, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    geometry_msgs::msg::TransformStamped NodeProxy::lookup_transform(const std::string &target_frame, const rclcpp::Time &target_time,
                                                                     const std::string &source_frame, const rclcpp::Time &source_time,
                                                                     const std::string &fixed_frame){
      try {
        return tf_buffer_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, tf2::durationFromSec(0.2));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return geometry_msgs::msg::TransformStamped();
      }
    }

    bool NodeProxy::transform_cloud(const sensor_msgs::msg::PointCloud2 & in_cloud, sensor_msgs::msg::PointCloud2 & out_cloud, const std::string &target_frame, bool deskew_lidar){
      try {
        if (deskew_lidar) {
        	deskew(in_cloud, out_cloud, target_frame, 1.0, false);
        }else {
          out_cloud = tf_buffer_->transform(in_cloud, target_frame, tf2::durationFromSec(0.2));
          //        tf2::doTransform(in_cloud, out_cloud, lookup_transform(target_frame, in_cloud.header.frame_id));
        }
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        out_cloud = in_cloud;
        return false;
      }
    }

    bool NodeProxy::transform_cloud(const sensor_msgs::msg::PointCloud2 & in_cloud, sensor_msgs::msg::PointCloud2 & out_cloud, 
                                    const std::string &target_frame, const rclcpp::Time &target_time, const std::string &fixed_frame) {
      try {
        tf_buffer_->transform(in_cloud, out_cloud, target_frame, tf2_ros::fromMsg(target_time), fixed_frame, tf2::durationFromSec(0.2));
//        tf2::doTransform(in_cloud, out_cloud, lookup_transform(target_frame, in_cloud.header.frame_id));
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform cloud %s to %s: %s", in_cloud.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        out_cloud = in_cloud;
        return false;
      }
    }

    bool NodeProxy::transform_pose(const geometry_msgs::msg::PoseStamped & in_pose, geometry_msgs::msg::PoseStamped & out_pose, const std::string &target_frame, float duration) {
      try {
        out_pose = tf_buffer_->transform(in_pose, target_frame, tf2::durationFromSec(duration));
        return true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "Could not transform pose %s to %s: %s", in_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        out_pose = in_pose;
        return false;
      }
    }

    rclcpp::Time NodeProxy::get_stamp() const {
      return node_->get_clock()->now();
    }

    double NodeProxy::get_now_seconds() const {
      return get_stamp().seconds();
    }

    void NodeProxy::spin_some() {
      rclcpp::spin_some(node_);
    }

    void NodeProxy::spin() {
      rclcpp::spin(node_);
    }


	bool NodeProxy::deskew(
			const sensor_msgs::msg::PointCloud2 & input,
			sensor_msgs::msg::PointCloud2 & output,
			const std::string & fixedFrameId,
			double waitForTransform,
			bool slerp)
	{

		if ( input.width == 0 || input.height == 0) {
			RCLCPP_WARN(node_->get_logger(), "Received empty point cloud.");
			return false;
		}

		int offsetTime = -1;
		int offsetX = -1;
		int offsetY = -1;
		int offsetZ = -1;
		int timeDatatype = 6;
		for(size_t i=0; i<input.fields.size(); ++i)
		{
			if(input.fields[i].name.compare("t") == 0 ||
			   input.fields[i].name.compare("time") == 0 ||
			   input.fields[i].name.compare("stamps") == 0 ||
			   input.fields[i].name.compare("timestamp") == 0)
			{
				if(offsetTime != -1)
				{
					RCLCPP_WARN(node_->get_logger(), "The input cloud should have only one of these fields: t, time, stamps or timestamp. Overriding with %s.", input.fields[i].name.c_str());
				}
				offsetTime = input.fields[i].offset;
				timeDatatype = input.fields[i].datatype;
			}
			else if(input.fields[i].name.compare("x") == 0)
			{
				offsetX = input.fields[i].offset;
			}
			else if(input.fields[i].name.compare("y") == 0)
			{
				offsetY = input.fields[i].offset;
			}
			else if(input.fields[i].name.compare("z") == 0)
			{
				offsetZ = input.fields[i].offset;
			}
		}

		if(offsetTime < 0)
		{
			return false;
		}

		bool timeOnColumns = input.width > input.height;

		// Get latest timestamp
		rclcpp::Time firstStamp;
		rclcpp::Time lastStamp;
		if(timeDatatype == 6) // UINT32
		{
			unsigned int nsecFirst = *((const unsigned int*)(&input.data[0]+offsetTime));
			unsigned int nsecLast = *((const unsigned int*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));

			if(nsecFirst > nsecLast)
			{
				RCLCPP_WARN_ONCE(node_->get_logger(), "Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add computation time.");
				if(timeOnColumns)
				{
					for(size_t i=0; i<input.width; ++i)
					{
						unsigned int nsec = *((const unsigned int*)(&input.data[(i)*input.point_step]+offsetTime));
						if(nsec < nsecFirst)
						{
							nsecFirst = nsec;
						}
						else if(nsec > nsecLast)
						{
							nsecLast = nsec;
						}
					}
				}
				else
				{
					for(size_t i=0; i<input.height; ++i)
					{
						unsigned int nsec = *((const unsigned int*)(&input.data[input.row_step*(i)]+offsetTime));
						if(nsec < nsecFirst)
						{
							nsecFirst = nsec;
						}
						else if(nsec > nsecLast)
						{
							nsecLast = nsec;
						}
					}
				}
			}

			firstStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsecFirst);
			lastStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsecLast);
		}
		else if(timeDatatype == 7) // FLOAT32
		{
			float secFirst = *((const float*)(&input.data[0]+offsetTime));
			float secLast = *((const float*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));

			if(secFirst > secLast)
			{
				// scans are not ordered, we need to search min/max
				RCLCPP_WARN_ONCE(node_->get_logger(), "Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add computation time.");
				if(timeOnColumns)
				{
					for(size_t i=0; i<input.width; ++i)
					{
						float sec = *((const float*)(&input.data[(i)*input.point_step]+offsetTime));
						if(sec < secFirst)
						{
							secFirst = sec;
						}
						else if(sec > secLast)
						{
							secLast = sec;
						}
					}
				}
				else
				{
					for(size_t i=0; i<input.height; ++i)
					{
						float sec = *((const float*)(&input.data[input.row_step*(i)]+offsetTime));
						if(sec < secFirst)
						{
							secFirst = sec;
						}
						else if(sec > secLast)
						{
							secLast = sec;
						}
					}
				}
			}

			firstStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(secFirst);
			lastStamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(secLast);
		}
		else if(timeDatatype == 8) // FLOAT64
		{
			double secFirst = *((const double*)(&input.data[0]+offsetTime));
			double secLast = *((const double*)(&input.data[(input.width-1)*input.point_step + input.row_step*(input.height-1)]+offsetTime));
			if(secFirst > secLast)
			{
				// scans are not ordered, we need to search min/max
				RCLCPP_WARN_ONCE(node_->get_logger(), "Timestamp channel is not ordered, we will have to parse every scans to "
					  "determinate first and last time offsets. This will add computation time.");
				if(timeOnColumns)
				{
					for(size_t i=0; i<input.width; ++i)
					{
						double sec = *((const double*)(&input.data[(i)*input.point_step]+offsetTime));
						if(sec < secFirst)
						{
							secFirst = sec;
						}
						else if(sec > secLast)
						{
							secLast = sec;
						}
					}
				}
				else
				{
					for(size_t i=0; i<input.height; ++i)
					{
						double sec = *((const double*)(&input.data[input.row_step*(i)]+offsetTime));
						if(sec < secFirst)
						{
							secFirst = sec;
						}
						else if(sec > secLast)
						{
							secLast = sec;
						}
					}
				}
			}

			if(secFirst > 1.e18)
			{
				// convert nanoseconds to seconds
				secFirst /= 1.e9;
				secLast /= 1.e9;
			}
			else if(secFirst > 1.e15)
			{
				// convert microseconds to seconds
				secFirst /= 1.e6;
				secLast /= 1.e6;
			}
			else if(secFirst > 1.e12)
			{
				// convert milliseconds to seconds
				secFirst /= 1.e3;
				secLast /= 1.e3;
			}

			firstStamp = timestampToROS(secFirst);
			lastStamp = timestampToROS(secLast);
		}
		else
		{
			RCLCPP_WARN_ONCE(node_->get_logger(), "Not supported time datatype %d!", timeDatatype);
			return false;
		}

		if(!(timeDatatype >=6 && timeDatatype<=8))
		{
			RCLCPP_WARN_ONCE(node_->get_logger(), "Only lidar timestamp channel data type 6, 7 or 8 is supported! (received %d)", timeDatatype);
			return false;
		}
		if(lastStamp < firstStamp)
		{
			RCLCPP_WARN_ONCE(node_->get_logger(), "Last stamp (%f) is smaller than first stamp (%f) (header=%f)!", timestampFromROS(lastStamp), timestampFromROS(firstStamp), timestampFromROS(input.header.stamp));
			return false;
		}
		else if(lastStamp == firstStamp)
		{
			RCLCPP_WARN_ONCE(node_->get_logger(), "First and last stamps in the scan are the same (%f) (header=%f)!", timestampFromROS(lastStamp), timestampFromROS(input.header.stamp));
			return false;
		}
		std::string errorMsg;
		if(waitForTransform>0.0 &&
		   !tf_buffer_->canTransform(
				input.header.frame_id,
				firstStamp,
				input.header.frame_id,
				lastStamp,
				fixedFrameId,
				rclcpp::Duration::from_seconds(waitForTransform),
				&errorMsg))
		{
			RCLCPP_WARN(node_->get_logger(), "Could not estimate motion of %s accordingly to fixed frame %s between stamps %f and %f! (%s)",
					input.header.frame_id.c_str(),
					fixedFrameId.c_str(),
					timestampFromROS(firstStamp),
					timestampFromROS(lastStamp),
					errorMsg.c_str());
			return false;
		}

		// rtabmap::Transform firstPose;
		// rtabmap::Transform lastPose;
		// double scanTime = 0;
		// if(slerp)
		// {
		// 	if(tfBuffer != 0)
		// 	{
		// 		firstPose = rtabmap_conversions::getMovingTransform(
		// 				input.header.frame_id,
		// 				fixedFrameId,
		// 				input.header.stamp,
		// 				firstStamp,
		// 				*tfBuffer,
		// 				0);
		// 		lastPose = rtabmap_conversions::getMovingTransform(
		// 				input.header.frame_id,
		// 				fixedFrameId,
		// 				input.header.stamp,
		// 				lastStamp,
		// 				*tfBuffer,
		// 				0);
		// 	}
		// 	else
		// 	{
		// 		float vx,vy,vz, vroll,vpitch,vyaw;
		// 		velocity.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);
		//
		// 		// We need three poses:
		// 		//  1- The pose of base frame in odom frame at first stamp
		// 		//  2- The pose of base frame in odom frame at msg stamp
		// 		//  3- The pose of base frame in odom frame at last stamp
		// 		UASSERT(timestampFromROS(firstStamp) >= previousStamp);
		// 		UASSERT(timestampFromROS(lastStamp) > previousStamp);
		// 		double dt1 = timestampFromROS(firstStamp) - previousStamp;
		// 		double dt2 = timestampFromROS(input.header.stamp) - previousStamp;
		// 		double dt3 = timestampFromROS(lastStamp) - previousStamp;
		//
		// 		rtabmap::Transform p1(vx*dt1, vy*dt1, vz*dt1, vroll*dt1, vpitch*dt1, vyaw*dt1);
		// 		rtabmap::Transform p2(vx*dt2, vy*dt2, vz*dt2, vroll*dt2, vpitch*dt2, vyaw*dt2);
		// 		rtabmap::Transform p3(vx*dt3, vy*dt3, vz*dt3, vroll*dt3, vpitch*dt3, vyaw*dt3);
		//
		// 		// First and last poses are relative to stamp of the msg
		// 		firstPose = p2.inverse() * p1;
		// 		lastPose = p2.inverse() * p3;
		// 	}
		//
		// 	if(firstPose.isNull())
		// 	{
		// 		UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
		// 				input.header.frame_id.c_str(),
		// 				fixedFrameId.empty()?"velocity":fixedFrameId.c_str(),
		// 				timestampFromROS(firstStamp),
		// 				timestampFromROS(input.header.stamp));
		// 		return false;
		// 	}
		// 	if(lastPose.isNull())
		// 	{
		// 		UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
		// 				input.header.frame_id.c_str(),
		// 				fixedFrameId.empty()?"velocity":fixedFrameId.c_str(),
		// 				timestampFromROS(lastStamp),
		// 				timestampFromROS(input.header.stamp));
		// 		return false;
		// 	}
		// 	scanTime = timestampFromROS(lastStamp) - timestampFromROS(firstStamp);
		// }
		//else tf will be used to get more accurate transforms

		output = input;
		rclcpp::Time stamp;
		if(timeOnColumns)
		{
			// ouster point cloud:
			// t1     t2    ...
			// ring1  ring1 ...
			// ring2  ring2 ...
			// ring3  ring4 ...
			// ring4  ring3 ...
			for(size_t u=0; u<output.width; ++u)
			{
				if(timeDatatype == 6) // UINT32
				{
					unsigned int nsec = *((const unsigned int*)(&output.data[u*output.point_step]+offsetTime));
					stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsec);
				}
				else if(timeDatatype == 7) //float 32
				{
					float sec = *((const float*)(&output.data[u*output.point_step]+offsetTime));
					stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(sec);
				}
				else if(timeDatatype == 8) //float64
				{
					double sec = *((const double*)(&output.data[u*output.point_step]+offsetTime));
					if(sec > 1.e18)
					{
						// convert nanoseconds to seconds
						sec /= 1.e9;
					}
					else if(sec > 1.e15)
					{
						// convert microseconds to seconds
						sec /= 1.e6;
					}
					else if(sec > 1.e12)
					{
						// sec milliseconds to seconds
						sec /= 1.e3;
					}
					stamp = timestampToROS(sec);
				}

				geometry_msgs::msg::TransformStamped transform;
				if(slerp)
				{
					// transform = firstPose.interpolate((stamp-firstStamp).seconds() / scanTime, lastPose);
				}
				else
				{
					transform = getMovingTransform(
							output.header.frame_id,
							fixedFrameId,
							output.header.stamp,
							stamp,
							0);
					// if(transform.isNull())
					// {
					// 	UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
					// 			output.header.frame_id.c_str(),
					// 			fixedFrameId.c_str(),
					// 			timestampFromROS(stamp),
					// 			timestampFromROS(output.header.stamp));
					// 	return false;
					// }
				}

				for(size_t v=0; v<input.height; ++v)
				{
					unsigned char * dataPtr = &output.data[v*output.row_step + u*output.point_step];
					float & x = *((float*)(dataPtr+offsetX));
					float & y = *((float*)(dataPtr+offsetY));
					float & z = *((float*)(dataPtr+offsetZ));
					KDL::Vector v_out = tf2::gmTransformToKDL(transform) * KDL::Vector(x, y, z);
					x = static_cast<float>(v_out.x());
					y = static_cast<float>(v_out.y());
					z = static_cast<float>(v_out.z());

					// set delta stamp to zero so that on downstream they know the cloud is deskewed
					if(timeDatatype == 6) // UINT32
					{
						*((unsigned int*)(dataPtr+offsetTime)) = 0;
					}
					else if(timeDatatype == 7)
					{
						*((float*)(dataPtr+offsetTime)) = 0;
					}
					else if(timeDatatype == 8)
					{
						*((double*)(dataPtr+offsetTime)) = 0;
					}
				}
			}
		}
		else // time on rows
		{
			// velodyne point cloud:
			// t1     ring1 ring2 ring3 ring4
			// t2     ring1 ring2 ring3 ring4
			// t3     ring1 ring2 ring3 ring4
			// t4     ring1 ring2 ring3 ring4
			// ...    ...   ...   ...   ...
			for(size_t v=0; v<output.height; ++v)
			{
				if(timeDatatype == 6) // UINT32
				{
					unsigned int nsec = *((const unsigned int*)(&output.data[v*output.row_step]+offsetTime));
					stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration(0, nsec);
				}
				else if(timeDatatype == 7) //float 32
				{
					float sec = *((const float*)(&output.data[v*output.row_step]+offsetTime));
					stamp = rclcpp::Time(input.header.stamp)+rclcpp::Duration::from_seconds(sec);
				}
				else if(timeDatatype == 8)
				{
					double sec = *((const double*)(&output.data[v*output.row_step]+offsetTime));
					if(sec > 1.e18)
					{
						// convert nanoseconds to seconds
						sec /= 1.e9;
					}
					else if(sec > 1.e15)
					{
						// convert microseconds to seconds
						sec /= 1.e6;
					}
					else if(sec > 1.e12)
					{
						// sec milliseconds to seconds
						sec /= 1.e3;
					}
					stamp = timestampToROS(sec);
				}

				geometry_msgs::msg::TransformStamped transform;
				if(slerp)
				{
					// transform = firstPose.interpolate((stamp-firstStamp).seconds() / scanTime, lastPose);
				}
				else
				{
					transform = getMovingTransform(
							output.header.frame_id,
							fixedFrameId,
							output.header.stamp,
							stamp,
							0);
					// if(transform.isNull())
					// {
					// 	UERROR("Could not get transform of %s accordingly to %s between stamps %f and %f!",
					// 			output.header.frame_id.c_str(),
					// 			fixedFrameId.c_str(),
					// 			timestampFromROS(stamp),
					// 			timestampFromROS(output.header.stamp));
					// 	return false;
					// }
				}

				for(size_t u=0; u<input.width; ++u)
				{
					unsigned char * dataPtr = &output.data[v*output.row_step + u*output.point_step];
					float & x = *((float*)(dataPtr+offsetX));
					float & y = *((float*)(dataPtr+offsetY));
					float & z = *((float*)(dataPtr+offsetZ));
					KDL::Vector v_out = tf2::gmTransformToKDL(transform) * KDL::Vector(x, y, z);
					x = static_cast<float>(v_out.x());
					y = static_cast<float>(v_out.y());
					z = static_cast<float>(v_out.z());

					// set delta stamp to zero so that on downstream they know the cloud is deskewed
					if(timeDatatype == 6) // UINT32
					{
						*((unsigned int*)(dataPtr+offsetTime)) = 0;
					}
					else if(timeDatatype == 7)
					{
						*((float*)(dataPtr+offsetTime)) = 0;
					}
					else if(timeDatatype == 8)
					{
						*((double*)(dataPtr+offsetTime)) = 0;
					}
				}
			}
		}
		return true;
	}

	geometry_msgs::msg::TransformStamped NodeProxy::getMovingTransform(
		const std::string & movingFrame,
		const std::string & fixedFrame,
		const rclcpp::Time & stampFrom,
		const rclcpp::Time & stampTo,
		double waitForTransform) const
	{
		// TF ready?
		geometry_msgs::msg::TransformStamped transform;
		try
		{
			transform =  tf_buffer_->lookupTransform(movingFrame, tf2_ros::fromMsg(stampFrom), movingFrame, tf2_ros::fromMsg(stampTo), fixedFrame, tf2::durationFromSec(waitForTransform));
		}
		catch(tf2::TransformException & ex)
		{
			RCLCPP_WARN(node_->get_logger(), "(getting transform movement of %s according to fixed %s) %s", movingFrame.c_str(), fixedFrame.c_str(), ex.what());
		}
		return transform;
	}


#endif

const std::string NodeType::LocalPlanner = "local_planner";
const std::string NodeType::GlobalPlanner = "global_planner";
const std::string NodeType::Control = "control";
const std::string NodeType::Perception = "perception";
const std::string NodeType::Mission = "mission";

} // namespace node
} // namespace avt_341
