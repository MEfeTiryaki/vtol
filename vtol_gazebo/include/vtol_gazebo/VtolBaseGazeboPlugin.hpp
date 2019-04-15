/*
 File name: VtolBaseGazeboPlugin.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 28.10.2018
 Date last modified: 17.03.2019
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
// Rviz markers
#include <visualization_msgs/MarkerArray.h>

// sensor msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// urdf
#include <urdf/model.h>

// TF
#include <tf/transform_broadcaster.h>
#include <vtol_gazebo/wrench_module/AerodynamicForce.hpp>
// Eigen
#include <Eigen/Dense>

#include "ros_node_utils/ros_node_utils.hpp"

#include "ros_gazebo_utils/GazeboModelPluginBase.hpp"
#include "ros_gazebo_utils/WrenchVisualizer.hpp"
#include "ros_gazebo_utils/WrenchLink.hpp"
namespace gazebo {

class VtolBaseGazeboPlugin : public GazeboModelPluginBase
{
 public:
  /*! \~english
   Constructor
   */
  VtolBaseGazeboPlugin();

  /*! \~english
   Destructor
   */
  virtual ~VtolBaseGazeboPlugin()
  {
  }
  ;

  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  virtual void OnUpdate() override;
 protected:

  virtual void create() override;

  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initialize() override;

  virtual void initializeLinkStructure() override;

  // Inits the ROS subscriber.
  virtual void initializeSubscribers() override;

  // Inits the ROS subscriber.
  virtual void initializePublishers() override;

  // Read simulation state.
  virtual void readSimulation() override;

  // Writes simulation state.
  virtual void writeSimulation() override;

  // Publishes Tf for visulaization in RViz
  virtual void publishTf() override;

  // Publish forces to be visualized in RViz
  virtual void publish() override;

  /*! \~english
   *  Publishes visulization markers
   */
  virtual void publishMarker();


  // Debug Bool
  bool debug_;

  // Ensures gazebo methods are called sequentially
  std::mutex mutex_;

  // Name of the robot.
  std::string robotName_;
  std::string linkName_;

  // Simulation values
  Eigen::Vector3d eulerAngles_;
  tf::Transform T_WB_;



  Eigen::Vector3d forceOnBodyInWorldFrame_;
  Eigen::Vector3d torqueOnBodyInWorldFrame_;

  // AERODYNAMIC PARAMETERS
  wrench::AerodynamicForce* aerodynamics_;

  wrench::WrenchVisualizer* visualizer_;

  wrench::WrenchLink* wrenchLink_;

}
;

}
