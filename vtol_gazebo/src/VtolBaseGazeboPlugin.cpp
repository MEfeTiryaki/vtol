/*
File name: VtolBaseGazeboPlugin.cpp
Author: Mehmet Efe Tiryaki
E-mail: m.efetiryaki@gmail.com
Date created: 28.10.2018
Date last modified: 31.10.2018
 */

#include "vtol_gazebo/VtolBaseGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
VtolBaseGazeboPlugin::VtolBaseGazeboPlugin()
    : nodeHandle_(),
      angleAileronRight_(0),
      angleAileronLeft_(0),
      angleElevatorRight_(0),
      angleElevatorLeft_(0),
      forceOnBodyInWorldFrame_(Eigen::Vector3d::Zero()),
      torqueOnBodyInWorldFrame_(Eigen::Vector3d::Zero())
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

  robotName_  = sdf->GetElement("robot_name")->Get<std::string>();
  if(debug_){std::cout<<"Robot Name : " << robotName_ << std::endl;}

  linkName_  = sdf->GetElement("link_name")->Get<std::string>();
  if(debug_){std::cout<<"link name : " << linkName_ << std::endl;}


  // READ PARAMETERS
  std::vector<double> dummy;
  int row ;
  int col ;
  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/ail")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/ail", dummy);
    ail_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        ail_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The ail couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alpha_wake")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alpha_wake", dummy );
    alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        alpha_wake_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The alpha_wake couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alpha")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alpha", dummy);
    alpha_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        alpha_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The alpha couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim_wake")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim_wake", dummy);
    alphalim_wake_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        alphalim_wake_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The alphalim_wake couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim", dummy);
    alphalim_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        alphalim_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The alphalim couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv_row_number", row);
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv", dummy);
    col = dummy.size()/row;
    cd_alpha_elv_ = Eigen::MatrixXd::Zero(row,col);
    for(int i = 0 ; i<dummy.size();i++){
        cd_alpha_elv_(i/col,i%col) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cd_alpha_elv_ couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_wake")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_wake", dummy);
    cd_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cd_alpha_wake_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cd_alpha_wake couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha", dummy);
    cd_alpha_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cd_alpha_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cd_alpha couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_ail")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_ail", dummy);
    cl_ail_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_ail_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_ail couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha_wake")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha_wake", dummy);
    cl_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_alpha_wake_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_alpha_wake couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha", dummy);
    cl_alpha_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_alpha_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_alpha couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_beta")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_beta", dummy);
    cl_beta_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_beta_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_beta couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_elv")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_elv", dummy);
    cl_elv_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_elv_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_elv couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_p")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_p", dummy);
    cl_p_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_p_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_p couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_r")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_r", dummy);
    cl_r_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cl_r_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cl_r couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha_wake")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha_wake", dummy);
    cm_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cm_alpha_wake_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cm_alpha_wake couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha", dummy);
    cm_alpha_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cm_alpha_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cm_alpha couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_elv")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_elv", dummy);
    cm_elv_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cm_elv_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cm_elv couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail_row_number", row);
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail", dummy);
    col = dummy.size()/row;
    cn_ail_ = Eigen::MatrixXd::Zero(row,col);
    for(int i = 0 ; i<dummy.size();i++){
        cn_ail_(i/col,i%col) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cn_ail couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_beta")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_beta", dummy);
    cn_beta_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cn_beta_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cn_beta couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_p")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_p", dummy);
    cn_p_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cn_p_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cn_p couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_r")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_r", dummy);
    cn_r_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cn_r_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cn_r couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cy_p")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cy_p", dummy );
    cy_p_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        cy_p_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The cy_p couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/ele")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/ele", dummy);
    ele_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        ele_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The ele couldn't be found");
  }

  if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/elelim")) {
    this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/elelim", dummy);
    elelim_ = Eigen::VectorXd::Zero(dummy.size());
    for(int i = 0 ; i<dummy.size();i++){
        elelim_(i) = dummy[i];
    }
  }
  else {
    ROS_INFO("The elelim couldn't be found");
  }
  /*
  std::cout << "ail_\n" << ail_.transpose() << std::endl ;
  std::cout << "alpha_wake_\n" << alpha_wake_.transpose() << std::endl ;
  std::cout << "alpha_\n" << alpha_.transpose() << std::endl ;
  std::cout << "alphalim_wake_\n" << alphalim_wake_.transpose() << std::endl ;
  std::cout << "alphalim_\n" << alphalim_.transpose() << std::endl ;
  std::cout << "cd_alpha_elv_\n" << cd_alpha_elv_ << std::endl ;
  std::cout << "cd_alpha_wake_\n" << cd_alpha_wake_.transpose() << std::endl ;
  std::cout << "cd_alpha_\n" << cd_alpha_.transpose() << std::endl ;
  std::cout << "cl_ail_\n" << cl_ail_.transpose() << std::endl ;
  std::cout << "cl_alpha_wake_\n" << cl_alpha_wake_.transpose() << std::endl ;
  std::cout << "cl_alpha_\n" << cl_alpha_.transpose() << std::endl ;
  std::cout << "cl_beta_\n" << cl_beta_.transpose() << std::endl ;
  std::cout << "cl_elv_\n" << cl_elv_.transpose() << std::endl ;
  std::cout << "cl_p_\n" << cl_p_.transpose() << std::endl ;
  std::cout << "cl_r_\n" << cl_r_.transpose() << std::endl ;
  std::cout << "cm_alpha_wake_\n" << cm_alpha_wake_.transpose() << std::endl ;
  std::cout << "cm_alpha_\n" << cm_alpha_.transpose() << std::endl ;
  std::cout << "cm_elv_\n" << cm_elv_.transpose() << std::endl ;
  std::cout << "cn_ail_\n" << cn_ail_ << std::endl ;
  std::cout << "cn_beta_\n" << cn_beta_.transpose() << std::endl ;
  std::cout << "cn_p_\n" << cn_p_.transpose() << std::endl ;
  std::cout << "cn_r_\n" << cn_r_.transpose() << std::endl ;
  std::cout << "cy_p_\n" << cy_p_.transpose() << std::endl ;
  std::cout << "ele_\n" << ele_.transpose() << std::endl ;
  std::cout << "elelim_\n" << elelim_.transpose() << std::endl ;
  */
}

// INITILIZATION

void VtolBaseGazeboPlugin::initLinkStructure()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  link_ = model_->GetLink(linkName_);
  if (link_ == NULL){
    std::cout<<" Couldn't find the link : " << linkName_ << std::endl;
  }
}

void VtolBaseGazeboPlugin::initSubscribers()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  aileronRightSubscriber_ = nodeHandle_->subscribe("/"+robotName_+"/AR/command",
                                            10,
                                            &VtolBaseGazeboPlugin::AileronRightCommandsCallback,
                                            this);
  aileronLeftSubscriber_= nodeHandle_->subscribe("/"+robotName_+"/AL/command",
                                            10,
                                            &VtolBaseGazeboPlugin::AileronLeftCommandsCallback,
                                            this);
  elevatorRightSubscriber_= nodeHandle_->subscribe("/"+robotName_+"/ER/command",
                                            10,
                                            &VtolBaseGazeboPlugin::ElevatorRightCommandsCallback,
                                            this);
  elevatorLeftSubscriber_ = nodeHandle_->subscribe("/"+robotName_+"/EL/command",
                                            10,
                                            &VtolBaseGazeboPlugin::ElevatorLeftCommandsCallback,
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
  link_->AddRelativeForce(math::Vector3( forceOnBodyInWorldFrame_[0]
                                       , forceOnBodyInWorldFrame_[1]
                                       , forceOnBodyInWorldFrame_[2]));
  link_->AddRelativeTorque(math::Vector3( torqueOnBodyInWorldFrame_[0]
                                      , torqueOnBodyInWorldFrame_[1]
                                      , torqueOnBodyInWorldFrame_[2]));

}

void VtolBaseGazeboPlugin::readSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  positionWorldToBase_ << link_->GetWorldPose().pos.x
                        , link_->GetWorldPose().pos.y
                        , link_->GetWorldPose().pos.z ;
  orientationWorldToBase_ << link_->GetWorldPose().rot.x
                           , link_->GetWorldPose().rot.y
                           , link_->GetWorldPose().rot.z
                           , link_->GetWorldPose().rot.w;

  linearVelocityWorldToBase_ << link_->GetWorldAngularVel().x
                             , link_->GetWorldAngularVel().y
                             , link_->GetWorldAngularVel().z;
  angularVelocityWorldToBase_ << link_->GetWorldLinearVel().x
                             , link_->GetWorldLinearVel().y
                             , link_->GetWorldLinearVel().z;


}

// AERODYNAMICS
void VtolBaseGazeboPlugin::calculateAerodynamics()
{
  /*
    The orientation, position, linear velocities and angular velocities
    are read from the simulation.
    The aileron and elevator angles are also known from subscriber
  */
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
}


// PUBLISHING METHODS
void VtolBaseGazeboPlugin::publishTF()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin( tf::Vector3( positionWorldToBase_[0]
                                  , positionWorldToBase_[1]
                                  , positionWorldToBase_[2]));
  tf::Quaternion q = tf::Quaternion( orientationWorldToBase_[0]
                                   , orientationWorldToBase_[1]
                                   , orientationWorldToBase_[2]
                                   , orientationWorldToBase_[3]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", linkName_));


}

void VtolBaseGazeboPlugin::publish()
{

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
