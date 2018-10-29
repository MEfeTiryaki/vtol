/*
 Author : M. Efe Tiryaki
 */

#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <math.h>
// required for std::this_thread
#include <thread>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/Element.hh>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
// geometry msgs
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
// std msgs
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

// sensor msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// urdf
#include <urdf/model.h>

// TF
#include <tf/transform_broadcaster.h>
// Eigen

#include <Eigen/Dense>

namespace gazebo {
class VtolBaseGazeboPlugin : public ModelPlugin
{
 public:
  // Constructor.
  VtolBaseGazeboPlugin();

  // Destructor.
  virtual ~VtolBaseGazeboPlugin();

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

  // Overrides Gazebo init function.
  virtual void Init();

  // Overrides Gazebo reset function.
  virtual void Reset();

  virtual void OnUpdate();

 protected:
  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initJointStructures() ;

  virtual void initLinkStructure() ;

  // Inits the ROS subscriber.
  virtual void initSubscribers() ;
  // Inits the ROS subscriber.
  virtual void initPublishers() ;

  // Read simulation state.
  virtual void readSimulation();

  // Writes simulation state.
  virtual void writeSimulation();

  // Debug Bool
   bool debug_;
  // Ros node
  std::string ns_;
  ros::NodeHandle* nodeHandle_;

  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;

  // Name of the robot.
  std::string robotName_;
  std::string linkName_;

  // Pulishers
  //ros::Publisher leftMotorAnglePublisher_;
  // Publisher names
  //std::string leftMotorAnglePublisherName_;
  // Publisher queue_size
  //int leftMotorAnglePublisherQueueSize_;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Robot links
  physics::LinkPtr link_;

  // Simulation values
  Eigen::Vector3d positionWorldToBase_;
  Eigen::Vector4d orientationWorldToBase_;
  Eigen::Vector3d linearVelocityWorldToBase_;
  Eigen::Vector3d angularVelocityWorldToBase_;


};

}