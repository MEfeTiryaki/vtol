/*
 File name: VtolBaseGazeboPlugin.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 28.10.2018
 Date last modified: 14.11.2018
 */

#include "vtol_gazebo/VtolBaseGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
VtolBaseGazeboPlugin::VtolBaseGazeboPlugin()
    : nodeHandle_(),
      debug_(false),
      angleAileronRight_(0),
      angleAileronLeft_(0),
      angleElevatorRight_(0),
      angleElevatorLeft_(0),
      forceOnBodyInWorldFrame_(Eigen::Vector3d::Zero()),
      torqueOnBodyInWorldFrame_(Eigen::Vector3d::Zero()),
      mass_(0.0),
      qbar_(0.0),
      qbar_wake_(0.0),
      rho_(0.0),
      S_wake_(0.0),
      mass_(1.0),
      velocity_(0.0),
      velocityEffective_(0.0),
      angleOfAttack_(0.0),
      angleOfAttackDot_(0.0),
      angleOfAttackEffective_(0.0),
      sideSlipAngle_(0.0)
{
}

void VtolBaseGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  ns_ = ros::this_node::getNamespace();

  nodeHandle_ = new ros::NodeHandle("~");

  // Note : check if this is placed correctly
  this->readParameters(sdf);

  // Model
  this->model_ = model;

  // initialize joint structure
  initJointStructures();
  initLinkStructure();
  // initialize ROS pub/sub/services
  initSubscribers();
  initPublishers();

  aerodynamics_.initilize(nodeHandle_);
  // reset simulation variables
  Reset();

  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VtolBaseGazeboPlugin::OnUpdate, this));
}

void VtolBaseGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  debug_ = sdf->GetElement("debug")->Get<bool>();

  robotName_ = sdf->GetElement("robot_name")->Get<std::string>();
  if (debug_) {
    std::cout << "Robot Name : " << robotName_ << std::endl;
  }

  linkName_ = sdf->GetElement("link_name")->Get<std::string>();
  if (debug_) {
    std::cout << "link name : " << linkName_ << std::endl;
  }

  // READ PARAMETERS
  aerodynamics_.readParameters();
}

// INITILIZATION

void VtolBaseGazeboPlugin::initLinkStructure()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  link_ = model_->GetLink(linkName_);
  if (link_ == NULL) {
    std::cout << " Couldn't find the link : " << linkName_ << std::endl;
  }
}

void VtolBaseGazeboPlugin::initSubscribers()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  aileronRightSubscriber_ = nodeHandle_->subscribe(
      "/" + robotName_ + "/AR/command", 10, &VtolBaseGazeboPlugin::AileronRightCommandsCallback,
      this);
  aileronLeftSubscriber_ = nodeHandle_->subscribe(
      "/" + robotName_ + "/AL/command", 10, &VtolBaseGazeboPlugin::AileronLeftCommandsCallback,
      this);
  elevatorRightSubscriber_ = nodeHandle_->subscribe(
      "/" + robotName_ + "/ER/command", 10, &VtolBaseGazeboPlugin::ElevatorRightCommandsCallback,
      this);
  elevatorLeftSubscriber_ = nodeHandle_->subscribe(
      "/" + robotName_ + "/EL/command", 10, &VtolBaseGazeboPlugin::ElevatorLeftCommandsCallback,
      this);
}

void VtolBaseGazeboPlugin::initPublishers()
{

}

// SIMULATION UPDATE
void VtolBaseGazeboPlugin::OnUpdate()
{
  readSimulation();
  writeSimulation();
  publishTF();
  publish();
}

// SIMULATION METHODS
void VtolBaseGazeboPlugin::writeSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  // Calculate Force and Moments
  calculateAerodynamics();

  // Add Forces and Moment on the Body
  link_->AddRelativeForce(
      math::Vector3(forceOnBodyInWorldFrame_[0], forceOnBodyInWorldFrame_[1],
                    forceOnBodyInWorldFrame_[2]));
  link_->AddRelativeTorque(
      math::Vector3(torqueOnBodyInWorldFrame_[0], torqueOnBodyInWorldFrame_[1],
                    torqueOnBodyInWorldFrame_[2]));

}

void VtolBaseGazeboPlugin::readSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  positionWorldToBase_ << link_->GetWorldPose().pos.x, link_->GetWorldPose().pos.y, link_
      ->GetWorldPose().pos.z;
  orientationWorldToBase_ << link_->GetWorldPose().rot.x, link_->GetWorldPose().rot.y, link_
      ->GetWorldPose().rot.z, link_->GetWorldPose().rot.w;

  linearVelocityWorldToBase_ << link_->GetWorldAngularVel().x, link_->GetWorldAngularVel().y, link_
      ->GetWorldAngularVel().z;
  angularVelocityWorldToBase_ << link_->GetWorldLinearVel().x, link_->GetWorldLinearVel().y, link_
      ->GetWorldLinearVel().z;

}

// PUBLISHING METHODS
void VtolBaseGazeboPlugin::publishTF()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  static tf::TransformBroadcaster br;

  T_WB_.setOrigin(
      tf::Vector3(positionWorldToBase_[0], positionWorldToBase_[1], positionWorldToBase_[2]));
  tf::Quaternion q = tf::Quaternion(orientationWorldToBase_[0], orientationWorldToBase_[1],
                                    orientationWorldToBase_[2], orientationWorldToBase_[3]);
  T_WB_.setRotation(q);
  br.sendTransform(tf::StampedTransform(T_WB_, ros::Time::now(), "odom", linkName_));

}

void VtolBaseGazeboPlugin::publish()
{

}

// AERODYNAMICS
void VtolBaseGazeboPlugin::calculateAerodynamics()
{
  /*
   The orientation, position, linear velocities and angular velocities
   are read from the simulation.
   The aileron and elevator angles are also known from subscriber
   */
  // XXX : comment in this section for debug purposes
  /*
   std::cout<< "Position     : " << positionWorldToBase_.transpose()<< std::endl;
   std::cout<< "Orientaiton  : " << orientationWorldToBase_.transpose()<< std::endl;
   std::cout<< "Lin Velocity : " << linearVelocityWorldToBase_.transpose()<< std::endl;
   std::cout<< "Ang Velocity : " << angularVelocityWorldToBase_.transpose()<< std::endl;


   std::cout<< "Angle Aileron Right  : " << angleAileronRight_<< std::endl;
   std::cout<< "Angle Aileron Left   : " << angleAileronLeft_<< std::endl;
   std::cout<< "Angle Elevator Right : " << angleElevatorRight_<< std::endl;
   std::cout<< "Angle Elevator Left  : " << angleElevatorLeft_<< std::endl;
   //*/

  // Todo : Calculate euler angles in rad from orientation
  // eulerAngles_ = ;
  // velocity of the base
  velocity_ = linearVelocityWorldToBase_.norm();

  // TODO Mehmet Efe Tiryaki 14.11.2018 : Calcualte VelocityEffective
  // radius of the region affected by propeller wake flow Rs
  // Area of that region As, qbar wake

  // TODO: Calculate propeller related variables
  //double S_wake = 0;

  // Calculate angle of attack and sideslip angle
  calculateAngleOfAttack();
  calculateSideSlip();

  // Athosphere values
  rho_ = 1.225;
  qbar_ = 0.5 * rho_ * velocity_ * velocity_;
  qbar_wake_ = 0.5 * rho_ * velocityEffective_ * velocityEffective_;

  // This method updates Aerodynamic coefficients
  aerodynamics_.updateAerodyamics(angleOfAttack_, angleOfAttackDot_, angleOfAttackEffective_,
                                  sideSlipAngle_, angularVelocityWorldToBase_[0],
                                  angularVelocityWorldToBase_[1], angularVelocityWorldToBase_[2],
                                  angleElevatorRight_, angleAileronRight_, velocity_,
                                  velocityEffective_);
  // Forces and Moment are calculated as 3D vectors
  F_wake0_ = aerodynamics_.getCFWake0() * qbar_ * S_wake_ / mass_;
  M_wake0_ = aerodynamics_.getCMWake0().cwiseProduct(aerodynamics_.getBWake()) * qbar_ * S_wake_;

  F_free_ = aerodynamics_.getCFWake0() * qbar_ * S_wake_ / mass_ - F_wake0_;
  M_free_ = aerodynamics_.getCMWake0().cwiseProduct(aerodynamics_.getB()) * qbar_ * S_wake_
      - M_wake0_;

  F_wake_ = aerodynamics_.getCFWake() * qbar_wake_ * S_wake_ / mass_;
  M_wake_ = aerodynamics_.getCMWake().cwiseProduct(aerodynamics_.getBWake()) * qbar_wake_ * S_wake_;

  // Final moments in Body Frame
  Eigen::Vector3d forceOnBodyInBodyFrame = F_free_ + F_wake_;
  Eigen::Vector3d momentOnBodyInBodyFrame = M_free_ + M_wake_;
}

Propeller VtolBaseGazeboPlugin::propellerParameterCalculate(double Throttle, double vt,
                                                            double tilt_angle,
                                                            double tilt_angle_rad, double alphar,
                                                            double betar, double rho)
{
  // TODO : calculate propeller parameters here
  return Propeller();
}

//

void VtolBaseGazeboPlugin::calculateAngleOfAttack()
{
  // TODO Mehmet Efe Tiryaki 14.11.2018 : calculate angle of attack here

}
void VtolBaseGazeboPlugin::calculateSideSlip()
{
  // TODO Mehmet Efe Tiryaki 14.11.2018 : calculate sideslip here

}

// CALLBACKS
void VtolBaseGazeboPlugin::AileronRightCommandsCallback(const std_msgs::Float64& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  angleAileronRight_ = msg.data;
}
void VtolBaseGazeboPlugin::AileronLeftCommandsCallback(const std_msgs::Float64& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  angleAileronLeft_ = msg.data;
}
void VtolBaseGazeboPlugin::ElevatorRightCommandsCallback(const std_msgs::Float64& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  angleElevatorRight_ = msg.data;

}
void VtolBaseGazeboPlugin::ElevatorLeftCommandsCallback(const std_msgs::Float64& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  angleElevatorLeft_ = msg.data;
}

GZ_REGISTER_MODEL_PLUGIN(VtolBaseGazeboPlugin)
}
