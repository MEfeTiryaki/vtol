/*
File name: VtolBaseGazeboPlugin.hpp
Author: Mehmet Efe Tiryaki
E-mail: m.efetiryaki@gmail.com
Date created: 28.10.2018
Date last modified: 31.10.2018
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
  virtual ~VtolBaseGazeboPlugin(){};

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

  // Overrides Gazebo init function.
  virtual void Init(){};

  // Overrides Gazebo reset function.
  virtual void Reset(){};

  virtual void OnUpdate();

 protected:
  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initJointStructures(){};

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

  //
  virtual void calculateAerodynamics();
  // CALLBACKS
  void AileronRightCommandsCallback(const std_msgs::Float64& msg);
  void AileronLeftCommandsCallback(const std_msgs::Float64& msg);
  void ElevatorRightCommandsCallback(const std_msgs::Float64& msg);
  void ElevatorLeftCommandsCallback(const std_msgs::Float64& msg);


  // Debug Bool
   bool debug_;
  // Ros node
  std::string ns_;
  ros::NodeHandle* nodeHandle_;

  // Ensures gazebo methods are called sequentially
   std::mutex mutex_;

  // Name of the robot.
  std::string robotName_;
  std::string linkName_;


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

  // Control surface angles
  double angleAileronRight_;
  double angleAileronLeft_;
  double angleElevatorRight_;
  double angleElevatorLeft_;
  // Subscriber
  ros::Subscriber aileronRightSubscriber_;
  ros::Subscriber aileronLeftSubscriber_;
  ros::Subscriber elevatorRightSubscriber_;
  ros::Subscriber elevatorLeftSubscriber_;


  // AERODYNAMIC FORCES and Moments
  Eigen::Vector3d forceOnBodyInWorldFrame_;
  Eigen::Vector3d torqueOnBodyInWorldFrame_;
  // AERODYNAMIC PARAMETERS
  /*
    ail_ ;
    alpha_wake_ ;
    alpha_ ;
    alphalim_wake_ ;
    alphalim_ ;
    cd_alpha_elv_ ;
    cd_alpha_wake_ ;
    cd_alpha_ ;
    cl_ail_ ;
    cl_alpha_wake_ ;
    cl_alpha_ ;
    cl_beta_ ;
    cd_alpha_elv_ ;
    cd_elv_ ;
    cl_p_ ;
    cl_r_ ;
    cm_alpha_wake_ ;
    cm_alpha_ ;
    cm_elv_ ;
    cn_ail_ ;
    cn_beta_ ;
    cn_p_ ;
    cn_r_ ;
    cy_p_ ;
    ele_ ;
    elelim_ ;
  */
  Eigen::VectorXd  ail_ ;
  Eigen::VectorXd alpha_wake_ ;
  Eigen::VectorXd alpha_ ;
  Eigen::VectorXd alphalim_wake_ ;
  Eigen::VectorXd alphalim_ ;
  Eigen::MatrixXd cd_alpha_elv_ ;
  Eigen::VectorXd cd_alpha_wake_ ;
  Eigen::VectorXd cd_alpha_ ;
  Eigen::VectorXd cl_ail_ ;
  Eigen::VectorXd cl_alpha_wake_ ;
  Eigen::VectorXd cl_alpha_ ;
  Eigen::VectorXd cl_beta_ ;
  Eigen::VectorXd cl_elv_ ;
  Eigen::VectorXd cl_p_ ;
  Eigen::VectorXd cl_r_ ;
  Eigen::VectorXd cm_alpha_wake_ ;
  Eigen::VectorXd cm_alpha_ ;
  Eigen::VectorXd cm_elv_ ;
  Eigen::MatrixXd  cn_ail_ ;
  Eigen::VectorXd cn_beta_ ;
  Eigen::VectorXd cn_p_ ;
  Eigen::VectorXd cn_r_ ;
  Eigen::VectorXd cy_p_ ;
  Eigen::VectorXd ele_ ;
  Eigen::VectorXd elelim_ ;

};

}
