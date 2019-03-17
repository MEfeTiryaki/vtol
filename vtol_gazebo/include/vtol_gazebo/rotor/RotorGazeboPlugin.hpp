/*
File name: RotorGazeboPlugin.hpp
Author: Mehmet Efe Tiryaki
E-mail: m.efetiryaki@gmail.com
Date created: 29.10.2018
Date last modified: 17.03.2019
 */
// TODO : Efe Tiryaki 17.03.2019 : This is old plugin structure
// imigrate to new GazeboPlugin structure

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


#include <visualization_msgs/Marker.h>

// urdf
#include <urdf/model.h>

// TF
#include <tf/transform_broadcaster.h>
// Eigen

#include <Eigen/Dense>


namespace gazebo {
class RotorGazeboPlugin : public ModelPlugin
{
 public:
  // Constructor.
  RotorGazeboPlugin();

  // Destructor.
  virtual ~RotorGazeboPlugin();

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

  virtual void publishTF();
  virtual void publish();

  void CommandsCallback(const std_msgs::Float64& msg);

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
  std::string jointName_;
  std::string rotorName_;
  std::string parentLinkName_;

  // Pulishers
  ros::Publisher visualizationPublisher_;
  // Publisher names
  //std::string leftMotorAnglePublisherName_;
  // Publisher queue_size
  //int leftMotorAnglePublisherQueueSize_;

  // Subscriber
  ros::Subscriber commandSubscriber_;
  // Subscriber names
  std::string commandSubscriberName_;
  // Subscriber queue_size
  int commandSubscriberQueueSize_;
  // Subscriber msgs
  std_msgs::Float64 command_;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Robot links
  physics::LinkPtr link_;
  physics::LinkPtr parentLink_;

  physics::JointPtr joint_;

  double simulationSpeedRatio_;

  tf::Transform T_rotor_blade_;
  tf::Transform T_base_rotor_;
  tf::Transform T_base_blade_;
  tf::Transform T_world_rotor_;


  double thrust_ ;
  double thrustCoefficient_;
  tf::Vector3 thrustVector_;
  tf::Transform rotorNormal_;

};

}
