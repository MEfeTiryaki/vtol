/*
File name: RotorGazeboPlugin.cpp
Author: Mehmet Efe Tiryaki
E-mail: m.efetiryaki@gmail.com
Date created: 29.10.2018
Date last modified: 17.03.2019
 */
// TODO : Efe Tiryaki 17.03.2019 : This is old plugin structure
// imigrate to new GazeboPlugin structure

#include "vtol_gazebo/rotor/RotorGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
RotorGazeboPlugin::RotorGazeboPlugin()
    : nodeHandle_(),
      command_()
{
}

RotorGazeboPlugin::~RotorGazeboPlugin()
{
}

void RotorGazeboPlugin::Init()
{
}
void RotorGazeboPlugin::Reset()
{
}

void RotorGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
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
      boost::bind(&RotorGazeboPlugin::OnUpdate, this));
}

void RotorGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  debug_ = sdf->GetElement("debug")->Get<bool>();

  robotName_  = sdf->GetElement("robot_name")->Get<std::string>();
  if(debug_){std::cout<<"Robot Name : " << robotName_ << std::endl;}

  linkName_  = sdf->GetElement("link_name")->Get<std::string>();
  if(debug_){std::cout<<"link name : " << linkName_ << std::endl;}

  rotorName_  = sdf->GetElement("rotor_name")->Get<std::string>();
  if(debug_){std::cout<<"rotor name : " << rotorName_ << std::endl;}

  jointName_  = sdf->GetElement("joint_name")->Get<std::string>();
  if(debug_){std::cout<<"joint name : " << jointName_ << std::endl;}


  simulationSpeedRatio_  = sdf->GetElement("simulation_speed_ratio")->Get<double>();
  if(debug_){std::cout<<"Simulation Speed Ratio : " << simulationSpeedRatio_ << std::endl;}

  thrustCoefficient_  = sdf->GetElement("thrust_coefficient")->Get<double>();
  if(debug_){std::cout<<"Thrust Coefficient : " << thrustCoefficient_ << std::endl;}

}



void RotorGazeboPlugin::initJointStructures()
{
  joint_ = model_->GetJoint(jointName_);
  if (joint_ == NULL){
    std::cout<<" Couldn't find the joint : " << jointName_ << std::endl;
  }else{
    joint_->SetVelocity(0, 0);
  }
}

void RotorGazeboPlugin::initLinkStructure()
{
  link_ = model_->GetLink(linkName_);
  parentLink_ = link_->GetParentJointsLinks().at(0);
  std::string parentLinkName_ = parentLink_->GetName();
  if (link_ == NULL){
    std::cout<<" Couldn't find the link : " << linkName_ << std::endl;
  }
}

void RotorGazeboPlugin::initSubscribers()
{
  commandSubscriber_ = nodeHandle_->subscribe("/"+robotName_+"/"+ rotorName_+"/command",
                                                        commandSubscriberQueueSize_,
                                                        &RotorGazeboPlugin::CommandsCallback,
                                                        this);
  command_.data = 0;

}

void RotorGazeboPlugin::initPublishers()
{
  visualizationPublisher_ = nodeHandle_->advertise<visualization_msgs::Marker>(
                          "/"+robotName_+"/"+ rotorName_+"thrust_marker", 0 );

}

void RotorGazeboPlugin::OnUpdate()
{
  readSimulation();
  writeSimulation();
  publishTF();
  publish();
}


void RotorGazeboPlugin::CommandsCallback(const std_msgs::Float64& msg)
{
  command_ = msg;
}
void RotorGazeboPlugin::writeSimulation()
{

  double velocity = simulationSpeedRatio_ * command_.data;
  joint_->SetVelocity(0, velocity);


  // APPLY TRUST ON ROTOR
  thrust_ = thrustCoefficient_ * command_.data * command_.data ;
  link_->AddRelativeForce(math::Vector3( 0,0,thrust_));

  // APPLY DRAG


}
void RotorGazeboPlugin::readSimulation()
{
  double yaw = joint_->GetAngle(0).Radian();
  math::Pose orientationRotorToBladeMath =  math::Pose(0,0,0,0,0,yaw)
                                      * parentLink_->GetInitialRelativePose().GetInverse()
                                      * link_->GetInitialRelativePose() ;
  T_rotor_blade_.setOrigin(tf::Vector3(orientationRotorToBladeMath.pos.x
                                      ,orientationRotorToBladeMath.pos.y
                                      ,orientationRotorToBladeMath.pos.z));
  T_rotor_blade_.setRotation(tf::Quaternion(orientationRotorToBladeMath.rot.x
                                           ,orientationRotorToBladeMath.rot.y
                                           ,orientationRotorToBladeMath.rot.z
                                           ,orientationRotorToBladeMath.rot.w));

}

void RotorGazeboPlugin::publishTF(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  br.sendTransform(tf::StampedTransform(
                    T_rotor_blade_
                  , ros::Time::now()
                  , link_->GetParentJointsLinks().at(0)->GetName()
                  , linkName_));

}

void RotorGazeboPlugin::publish(){
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time();
  marker.header.frame_id = linkName_ ;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0 ;
  marker.pose.position.y = 0.0 ;
  marker.pose.position.z = 0.0 ;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = -sin(0.5);
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = sin(0.5);
  marker.scale.x = 1.0  * thrust_;
  marker.scale.y = 0.01 ;
  marker.scale.z = 0.01 ;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  visualizationPublisher_.publish( marker );
}


GZ_REGISTER_MODEL_PLUGIN(RotorGazeboPlugin)
}
