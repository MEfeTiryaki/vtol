/*
 File name: VtolBaseGazeboPlugin.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 28.10.2018
 Date last modified: 14.11.2018
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

#include "vtol_gazebo/VtolAerodynamicContainer.hpp"

namespace gazebo {

struct Propeller
{
  double Omega;
  double lambda;
  double Vi;
  double Vw;
  double Thrust;
  double Rs;
  double Vts;
  double alpha_eff;
  double Torque;
};

class VtolBaseGazeboPlugin : public ModelPlugin
{
 public:
  // Constructor.
  VtolBaseGazeboPlugin();

  // Destructor.
  virtual ~VtolBaseGazeboPlugin()
  {
  }
  ;

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

  // Overrides Gazebo init function.
  virtual void Init()
  {
  }
  ;

  // Overrides Gazebo reset function.
  virtual void Reset()
  {
  }
  ;

  virtual void OnUpdate();

 protected:
  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initJointStructures()
  {
  }
  ;

  virtual void initLinkStructure();

  // Inits the ROS subscriber.
  virtual void initSubscribers();

  // Inits the ROS subscriber.
  virtual void initPublishers();

  // Read simulation state.
  virtual void readSimulation();

  // Writes simulation state.
  virtual void writeSimulation();

  // Publishes Tf for visulaization in RViz
  virtual void publishTF();

  // Publish forces to be visualized in RViz
  virtual void publish();

  // Calculates Aerodynamics using position/orientation and velocity
  // data from simulation
  virtual void calculateAerodynamics();

  // Calculates Angle of Attack and  Sideslip Angle
  void calculateAngles();


  virtual Propeller propellerParameterCalculate(double Throttle, double vt, double tilt_angle,
                                                double tilt_angle_rad, double alphar, double betar,
                                                double rho);

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
  Eigen::Vector3d eulerAngles_;
  tf::Transform T_WB_;

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
  Eigen::Vector3d F_free_;
  Eigen::Vector3d F_wake_;
  Eigen::Vector3d F_wake0_;
  Eigen::Vector3d M_free_;
  Eigen::Vector3d M_wake_;
  Eigen::Vector3d M_wake0_;

  Eigen::Vector3d forceOnBodyInWorldFrame_;
  Eigen::Vector3d torqueOnBodyInWorldFrame_;

  // AERODYNAMIC PARAMETERS
  VtolAerodynamicContainer aerodynamics_;

  double mass_;
  double qbar_;
  double qbar_wake_;
  double rho_;
  double S_wake_ ;
  double velocity_;
  double velocityEffective_;
  // Angle of Attack
  double angleOfAttack_;
  double angleOfAttackDot_ ;
  double angleOfAttackEffective_ ;
  // Side Slip Angle
  double sideSlipAngle_ ;
}
;

}
