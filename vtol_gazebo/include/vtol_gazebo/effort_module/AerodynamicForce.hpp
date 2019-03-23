/*
 File name: AerodynamicContainerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 10.11.2018
 Date last modified: 17.03.2019
 */

#pragma once

// c++
#include <mutex>
#include <string>
#include <vector>
#include <math.h>

#include "ros_node_base/EffortModuleBase.hpp"
#include "std_msgs/Float64.h"

using namespace ros_node_utils;

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

namespace effort {
class AerodynamicForce : public EffortModuleBase
{
 public:
  AerodynamicForce(ros::NodeHandle* nodeHandle)
      : EffortModuleBase(nodeHandle),
        b(1.619), /* span, m */
        b_wake(0.811),
        S(0.418), /* planform area, m^2 */
        cbar(0.272), /* mean aero chord, m */
        cbar_wake(0.305), /* mean aero chord of the region after propeller, m */
        Jy(0.100230), /* kg-m^2 */
        Jxz(-0.006363), /* kg-m^2 */
        Jz(0.475477), /* kg-m^2 */
        Jx(0.379385), /* kg-m^2 */
        ail_(),
        alpha_wake_(),
        alpha_(),
        alphalim_wake_(),
        alphalim_(),
        cd_alpha_elv_(),
        cd_alpha_wake_(),
        cd_alpha_(),
        cl_ail_(),
        cl_alpha_wake_(),
        cl_alpha_(),
        cl_beta_(),
        cl_elv_(),
        cl_p_(),
        cl_r_(),
        cm_alpha_wake_(),
        cm_alpha_(),
        cm_elv_(),
        cn_ail_(),
        cn_beta_(),
        cn_p_(),
        cn_r_(),
        cy_p_(),
        ele_(),
        elelim_(),
        CL_0(),
        CL_q(),
        CL_alpha_dot(0),
        CL_elv(0),
        CD_0(0),
        CD_elv(0),
        CM_0(0),
        CM_q(0),
        CM_ad(0),
        CM_elv(0),
        CY_beta(0),
        CY_p(0),
        CY_r(0),
        CY_ail(0),
        Cl_beta(0),
        Cl_p(0),
        Cl_r(0),    //%%%%%%%%%%%% !!!!!!!!!!
        Cl_ail(0),
        Cl_rud(0),
        CN_beta(0),
        CN_p(0),    //%%%%%%%%%%%% !!!!!!!!!!
        CN_r(0),
        CN_ail(0),
        CN_rud(0),
        CL_0_wake0(0),
        CL_q_wake0(0),
        CL_alpha_dot_wake0(0),
        CD0_wake0(0),
        CM_0_wake0(0),
        CM_q_wake0(0),
        CM_ad_wake0(0),
        CY_beta_wake0(0),
        CY_p_wake0(0),
        CY_r_wake0(0),
        Cl_beta_wake0(0),
        Cl_p_wake0(0),
        Cl_r_wake0(0),
        CN_beta_wake0(0),
        CN_p_wake0(0),
        CN_r_wake0(0),
        CL_0_wake(0),
        CL_q_wake(0),
        CL_alpha_dot_wake(0),
        CD0_wake(0),
        CM_0_wake(0),
        CM_q_wake(0),
        CM_ad_wake(0),
        CY_beta_wake(0),
        CY_p_wake(0),
        CY_r_wake(0),
        Cl_beta_wake(0),
        Cl_p_wake(0),
        Cl_r_wake(0),
        CN_beta_wake(0),
        CN_p_wake(0),
        CN_r_wake(0),
        CL_wake(0),
        CD_wake(0),
        CD_wake0(0),
        CL_wake0(0),
        CD0(0),
        CL(0),
        CD(0),
        angleAileronRight_(0),
        angleAileronLeft_(0),
        angleElevatorRight_(0),
        angleElevatorLeft_(0),
        mass_(0.0),
        qbar_(0.0),
        qbar_wake_(0.0),
        rho_(0.0),
        S_wake_(0.0),
        velocity_(0.0),
        velocityEffective_(0.0),
        angleOfAttack_(0.0),
        angleOfAttackDot_(0.0),
        angleOfAttackEffective_(0.0),
        sideSlipAngle_(0.0)

  {
  }
  ;

  virtual ~AerodynamicForce()
  {
  }
  ;

  virtual void readParameters()
  {
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/ail",ail_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/alpha_wake",alpha_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/alpha",alpha_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/alphalim_wake",alphalim_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/alphalim",alphalim_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cd_alpha_wake",cd_alpha_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cd_alpha",cd_alpha_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_ail",cl_ail_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_alpha_wake",cl_alpha_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_alpha",cl_alpha_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_beta",cl_beta_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_elv",cl_elv_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_p",cl_p_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cl_r",cl_r_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cm_alpha_wake",cm_alpha_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cm_alpha",cm_alpha_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cm_elv",cm_elv_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cm_alpha_wake",cm_alpha_wake_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cn_beta",cn_beta_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cn_p",cn_p_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cn_r",cn_r_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cy_p",cy_p_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/ele",ele_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/elelim",elelim_);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cd_alpha_elv",cd_alpha_elv_,7);
    paramRead(this->nodeHandle_,this->namespace_ + "/vtol_0001/aerodynamics/cn_ail",cn_ail_,9);

    //*
    std::cout << "ail_\n" << ail_.transpose() << std::endl;
    std::cout << "alpha_wake_\n" << alpha_wake_.transpose() << std::endl;
    std::cout << "alpha_\n" << alpha_.transpose() << std::endl;
    std::cout << "alphalim_wake_\n" << alphalim_wake_.transpose() << std::endl;
    std::cout << "alphalim_\n" << alphalim_.transpose() << std::endl;
    std::cout << "cd_alpha_elv_\n" << cd_alpha_elv_ << std::endl;
    std::cout << "cd_alpha_wake_\n" << cd_alpha_wake_.transpose() << std::endl;
    std::cout << "cd_alpha_\n" << cd_alpha_.transpose() << std::endl;
    std::cout << "cl_ail_\n" << cl_ail_.transpose() << std::endl;
    std::cout << "cl_alpha_wake_\n" << cl_alpha_wake_.transpose() << std::endl;
    std::cout << "cl_alpha_\n" << cl_alpha_.transpose() << std::endl;
    std::cout << "cl_beta_\n" << cl_beta_.transpose() << std::endl;
    std::cout << "cl_elv_\n" << cl_elv_.transpose() << std::endl;
    std::cout << "cl_p_\n" << cl_p_.transpose() << std::endl;
    std::cout << "cl_r_\n" << cl_r_.transpose() << std::endl;
    std::cout << "cm_alpha_wake_\n" << cm_alpha_wake_.transpose() << std::endl;
    std::cout << "cm_alpha_\n" << cm_alpha_.transpose() << std::endl;
    std::cout << "cm_elv_\n" << cm_elv_.transpose() << std::endl;
    std::cout << "cn_ail_\n" << cn_ail_ << std::endl;
    std::cout << "cn_beta_\n" << cn_beta_.transpose() << std::endl;
    std::cout << "cn_p_\n" << cn_p_.transpose() << std::endl;
    std::cout << "cn_r_\n" << cn_r_.transpose() << std::endl;
    std::cout << "cy_p_\n" << cy_p_.transpose() << std::endl;
    std::cout << "ele_\n" << ele_.transpose() << std::endl;
    std::cout << "elelim_\n" << elelim_.transpose() << std::endl;
    //*/
    CONFIRM("[AerodynamicForce] : read parameters");
  }
  ;

  void initializeSubscribers()
  {
    aileronRightSubscriber_ = this->nodeHandle_->subscribe(
        "/" + name_ + "/AR/command", 10, &AerodynamicForce::AileronRightCommandsCallback, this);
    aileronLeftSubscriber_ = this->nodeHandle_->subscribe(
        "/" + name_ + "/AL/command", 10, &AerodynamicForce::AileronLeftCommandsCallback, this);
    elevatorRightSubscriber_ = this->nodeHandle_->subscribe(
        "/" + name_ + "/ER/command", 10, &AerodynamicForce::ElevatorRightCommandsCallback, this);
    elevatorLeftSubscriber_ = this->nodeHandle_->subscribe(
        "/" + name_ + "/EL/command", 10, &AerodynamicForce::ElevatorLeftCommandsCallback, this);

    CONFIRM("[AerodynamicForce] : initialized Subscribers");
  }

  virtual Eigen::Vector3d getForce() override
  {
    calculateAerodynamics();
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    return  force_;
  }
  ;

  virtual Eigen::Vector3d getTorque() override
  {
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    return torque_;
  }
  ;

  virtual void updateAerodyamics(double alpha, double alphadot, double alphaEffective, double beta,
                                 double P, double Q, double R, double elevator, double aileron,
                                 double vt, double Vts)
  {
    CL_0 = interpolate(cl_alpha_, alpha_, alpha);  // _CL_alpha(alpha)
    // TODO Mehmet Efe Tiryaki 14.11.2018 : where is this defined
    //???CL_q = interpolate(cl_alpha_,alpha_,alpha); //_CL_Q(alpha);
    CL_alpha_dot = 0.0;
    CL_elv = interpolate(cl_elv_, ele_, elevator);  //_CL_elv(el);

    CD_0 = interpolate(cd_alpha_, alpha_, alpha);  //_CD_alpha(alpha);
    CD_elv = interpolate(cd_alpha_elv_, alpha_, ele_, alpha, elevator);

    CM_0 = interpolate(cm_alpha_, alpha_, alpha);  // _CM_alpha(alpha);
    // TODO Mehmet Efe Tiryaki 14.11.2018 : where is this defined
    //??? CM_q  = _CM_Q(alpha);
    // TODO Mehmet Efe Tiryaki 14.11.2018 : where is this defined
    //?? CM_ad  = _CM_ad(alpha);
    CM_elv = interpolate(cm_elv_, ele_, elevator);  //_CM_elv(el);

    // ?? _CY_beta(alpha);
    CY_p = interpolate(cy_p_, alpha_, alpha);  //_CY_p(alpha);
    CY_r = 0.0;
    CY_ail = 0.0;

    Cl_beta = interpolate(cl_beta_, alpha_, alpha);  //_Cl_beta(alpha);
    Cl_p = interpolate(cl_p_, alpha_, alpha);  //_Cl_p(alpha);
    Cl_r = interpolate(cl_r_, alpha_, alpha);  //_Cl_r(alpha);    //%%%%%%%%%%%% !!!!!!!!!!

    Cl_ail = interpolate(cl_ail_, ail_, aileron);  //_Cl_ail(ail);
    Cl_rud = 0.0;

    CN_beta = interpolate(cn_beta_, alpha_, alpha);  //_CN_beta(alpha);
    CN_p = interpolate(cn_p_, alpha_, alpha);  //_CN_p(alpha);    //%%%%%%%%%%%% !!!!!!!!!!
    CN_r = interpolate(cn_r_, alpha_, alpha);  //_CN_r(alpha);
    CN_ail = interpolate(cn_ail_, alpha_, ail_, alpha, aileron);  //_CN_ail(alpha,ail);
    CN_rud = 0.0;

    CL_0_wake0 = interpolate(cl_alpha_wake_, alpha_, alpha);  //_CL_alpha_wake(alpha);
    CL_q_wake0 = CL_q;
    CL_alpha_dot_wake0 = CL_alpha_dot;

    CD0_wake0 = interpolate(cd_alpha_wake_, alpha_, alpha);  // _CD_alpha_wake(alpha);
    CM_0_wake0 = interpolate(cm_alpha_wake_, alpha_, alpha);  //_CM_alpha_wake(alpha);
    CM_q_wake0 = CM_q;
    CM_ad_wake0 = CM_ad;

    CY_beta_wake0 = CY_beta;
    CY_p_wake0 = CY_p;
    CY_r_wake0 = CY_r;

    Cl_beta_wake0 = Cl_beta;
    Cl_p_wake0 = Cl_p;
    Cl_r_wake0 = Cl_r;

    CN_beta_wake0 = CN_beta;
    CN_p_wake0 = CN_p;
    CN_r_wake0 = CN_r;

    CL_0_wake = interpolate(cl_alpha_wake_, alpha_, alphaEffective);  // _CL_alpha_wake(alpha_eff);
    CL_q_wake = CL_q;
    CL_alpha_dot_wake = CL_alpha_dot;

    CD0_wake = interpolate(cd_alpha_wake_, alpha_, alphaEffective);  //_CD_alpha_wake(alpha_eff);

    CM_0_wake = interpolate(cm_alpha_wake_, alpha_, alphaEffective);  //_CM_alpha_wake(alpha_eff);
    CM_q_wake = CM_q;
    CM_ad_wake = CM_ad;

    CY_beta_wake = CY_beta;
    CY_p_wake = CY_p;
    CY_r_wake = CY_r;

    Cl_beta_wake = Cl_beta;
    Cl_p_wake = Cl_p;
    Cl_r_wake = Cl_r;

    CN_beta_wake = CN_beta;
    CN_p_wake = CN_p;
    CN_r_wake = CN_r;

    B_wake << b_wake, cbar_wake, b_wake;
    B << b, cbar, b;

    // TODO Mehmet Efe TIRYAKI 5.11.2018: WHY 0.1 is hard-coded ???
    CL = CL_0 + CL_elv + CL_q * Q * cbar / 2 / vt
        + CL_alpha_dot * alphadot * M_PI / 180 * cbar / 2 / vt;
    CD = CD0 + 0.1 + CD_elv;

    CF << -CD * cos(alpha * M_PI / 180) + CL * sin(alpha * M_PI / 180), CY_beta * beta * M_PI / 180
        + CY_p * P * b / 2 / vt + CY_r * R * b / 2 / vt + CY_ail, -CD * sin(alpha * M_PI / 180)
        - CL * cos(alpha * M_PI / 180);
    CM << Cl_beta * beta * M_PI / 180 + Cl_p * P * b / 2 / vt + Cl_r * R * b / 2 / vt + Cl_ail, CM_0
        + CM_elv + CM_q * Q * cbar / 2 / vt + CM_ad * alphadot * M_PI / 180 * cbar / 2 / vt, CN_beta
        * beta * M_PI / 180 + CN_p * P * b / 2 / vt + CN_r * R * b / 2 / vt + CN_ail;

    CL_wake0 = CL_0_wake0 + CL_q_wake0 * Q * cbar_wake / 2 / vt
        + CL_alpha_dot_wake0 * alphadot * M_PI / 180 * cbar_wake / 2 / vt;
    CD_wake0 = CD0_wake0;

    CF_wake0 << -CD_wake0 * cos(alpha * M_PI / 180) + CL_wake0 * sin(alpha * M_PI / 180), CY_beta_wake0
        * beta * M_PI / 180 + CY_p_wake0 * P * b_wake / 2 / vt + CY_r_wake0 * R * b_wake / 2 / vt, -CD_wake0
        * sin(alpha * M_PI / 180) - CL_wake0 * cos(alpha * M_PI / 180);
    CM_wake0
        << Cl_beta_wake0 * beta * M_PI / 180 + Cl_p_wake0 * P * b_wake / 2 / vt
            + Cl_r_wake0 * R * b_wake / 2 / vt, CM_0_wake0 + CM_q_wake0 * Q * cbar_wake / 2 / vt
        + CM_ad_wake0 * alphadot * M_PI / 180 * cbar_wake / 2 / vt, CN_beta_wake0 * beta * M_PI
        / 180 + CN_p_wake0 * P * b_wake / 2 / vt + CN_r_wake0 * R * b_wake / 2 / vt;

    CL_wake = CL_0_wake + CL_q_wake * Q * cbar_wake / 2 / Vts
        + CL_alpha_dot_wake * alphadot * M_PI / 180 * cbar_wake / 2 / Vts;
    CD_wake = CD0_wake;

    CF_wake << -CD_wake * cos(alpha * M_PI / 180) + CL_wake * sin(alpha * M_PI / 180), CY_beta_wake
        * beta * M_PI / 180 + CY_p_wake * P * b_wake / 2 / Vts + CY_r_wake * R * b_wake / 2 / Vts, -CD_wake
        * sin(alpha * M_PI / 180) - CL_wake * cos(alpha * M_PI / 180);
    CM_wake
        << Cl_beta_wake * beta * M_PI / 180 + Cl_p_wake * P * b_wake / 2 / Vts
            + Cl_r_wake * R * b_wake / 2 / Vts, CM_0_wake + CM_q_wake * Q * cbar_wake / 2 / Vts
        + CM_ad_wake * alphadot * M_PI / 180 * cbar_wake / 2 / Vts, CN_beta_wake * beta * M_PI / 180
        + CN_p_wake * P * b_wake / 2 / Vts + CN_r_wake * R * b_wake / 2 / Vts;

  }
  ;

  void calculateAerodynamics()
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

    // Todo Mehmet Efe Tiryaki 14.11.2018 : Calculate euler angles in rad from orientation
    // eulerAngles_ = ;
    // velocity of the base
    velocity_ = linearVelocity_.norm();

    // TODO Mehmet Efe Tiryaki 14.11.2018 : Calcualte VelocityEffective
    // radius of the region affected by propeller wake flow Rs
    // Area of that region As, qbar wake

    // TODO: Calculate propeller related variables
    //double S_wake = 0;

    // Calculate angle of attack and sideslip angle
    calculateAngles();

    // Athosphere values
    rho_ = 1.225;
    qbar_ = 0.5 * rho_ * velocity_ * velocity_;
    qbar_wake_ = 0.5 * rho_ * velocityEffective_ * velocityEffective_;

    // This method updates Aerodynamic coefficients
    updateAerodyamics(angleOfAttack_, angleOfAttackDot_, angleOfAttackEffective_,
                                    sideSlipAngle_, angularVelocity_[0],
                                    angularVelocity_[1], angularVelocity_[2],
                                    angleElevatorRight_, angleAileronRight_, velocity_,
                                    velocityEffective_);

    // Forces and Moment are calculated as 3D vectors
    F_wake0_ = CF_wake0 * qbar_ * S_wake_ / mass_;
    M_wake0_ = CM_wake0.cwiseProduct(B_wake) * qbar_ * S_wake_;

    F_free_ = CF_wake0 * qbar_ * S_wake_ / mass_ - F_wake0_;
    M_free_ = CM_wake0.cwiseProduct(B) * qbar_ * S_wake_
        - M_wake0_;

    F_wake_ = CF_wake * qbar_wake_ * S_wake_ / mass_;
    M_wake_ = CM_wake.cwiseProduct(B_wake) * qbar_wake_
        * S_wake_;

    std::cout << "F_wake  : " << F_wake_.transpose() << std::endl;
    // Final moments in Body Frame
    Eigen::Vector3d forceOnBodyInBodyFrame = F_free_ + F_wake_;
    Eigen::Vector3d momentOnBodyInBodyFrame = M_free_ + M_wake_;
  }


  void calculateAngles()
  {
    // TODO Mehmet Efe Tiryaki 16.11.2018 : Check conventions
    // XXX Mehmet Efe Tiryaki 16.11.2018 : This code assumes that the heading of the plane plane is in same
    // direction with heading of the wings

    // Unit direction vectors according to Aerospace conventions
    // Todo Mehmet Efe Tiryaki 16.11.2018 : Check Why rotation of a unit vector is not
    // a unit vector

    Eigen::Vector3d heading = (orientation_ * Eigen::Vector3d(1.0, 0.0, 0.0)).normalized();
    Eigen::Vector3d rightWing = (orientation_ * Eigen::Vector3d(0.0, -1.0, 0.0)).normalized();
    Eigen::Vector3d zDirection = (orientation_ * Eigen::Vector3d(0.0, 0.0, -1.0)).normalized();
    Eigen::Vector3d rightWingPlaneNormal = rightWing.cross(zDirection).normalized();

    Eigen::Vector3d velocity = linearVelocity_;

    // Right wing vector is the normal of the plane containing heading and z-direction
    Eigen::Vector3d velocityOnHeadingPlane = velocity - velocity.dot(rightWing) * rightWing;

    // Heading vector is not necessarily normal of the Right wing plane
    Eigen::Vector3d velocityOnRightWingPlane = velocity
        - velocity.dot(rightWingPlaneNormal) * rightWingPlaneNormal;

    // a.b = |a|*|b|*cos(theta) and
    if (velocity.norm() != 0) {
      angleOfAttack_ = asin((velocityOnHeadingPlane.normalized().cross(heading)).norm());
      sideSlipAngle_ = asin(velocityOnRightWingPlane.normalized().cross(rightWing).norm());
    } else {
      // if velocity is zero then velocity vector is in heading direction of the plane
      angleOfAttack_ = asin(
          ((orientation_ * Eigen::Vector3d(1.0, 0.0, 0.0)).normalized().cross(heading)).norm());
      sideSlipAngle_ = 0.0;
    }

    std::cout << "angleOfAttack_  : " << angleOfAttack_ << std::endl;
    std::cout << "sideSlipAngle_  : " << sideSlipAngle_ << std::endl;
    /*/ Debug
     std::cout << "velocity :" << velocity.length() << std::endl;
     std::cout << "velocity :" << velocity.getX() << " , " << velocity.getY() << " , "
     << velocity.getZ() << " , " << std::endl;
     std::cout << "velocityOnHeadingPlane :" << velocityOnHeadingPlane.getX() << " , "
     << velocityOnHeadingPlane.getY() << " , " << velocityOnHeadingPlane.getZ() << " , "
     << std::endl;
     std::cout << "velocityOnRightWingPlane :" << velocityOnRightWingPlane.getX() << " , "
     << velocityOnRightWingPlane.getY() << " , " << velocityOnRightWingPlane.getZ() << " , "
     << std::endl;
     std::cout << "heading :" << heading.getX() << " , " << heading.getY() << " , " << heading.getZ()
     << " , " << std::endl;
     std::cout << "rightWing :" << rightWing.getX() << " , " << rightWing.getY() << " , "
     << rightWing.getZ() << " , " << std::endl;
     std::cout << "zDirection :" << zDirection.getX() << " , " << zDirection.getY() << " , "
     << zDirection.getZ() << " , " << std::endl;
     std::cout << "Angle of attack :" << angleOfAttack_ << std::endl;
     std::cout << "Sideslip angle :" << sideSlipAngle_ << std::endl;
     std::cout << " _________________________" << std::endl;
     //*/
  }

 protected:

  double interpolate(Eigen::VectorXd dataY, Eigen::VectorXd dataX, double value)
  {
    if (value > dataX[dataX.size() - 1]) {
      std::cout << "Value : " << value << " is exceed max : " << dataX[dataX.size() - 1] << "!!\n";
      return -1000;
    }
    if (value < dataX[0]) {
      std::cout << "Value : " << value << " is exceed min : " << dataX[0] << "!!\n";
      return -1000;
    }
    // search index corresponding to correct section of X value
    int index = 0;
    while (index < dataX.size()) {
      if (dataX[index] > value) {
        break;
      }
      index++;
    }

    if (index == dataX.size()) {
      // return max value
      return dataY[index - 1];
    } else {
      // return interpolation
      return dataY[index - 1]
          + (dataY[index] - dataY[index - 1]) * (value - dataX[index - 1])
              / (dataX[index] - dataX[index - 1]);
    }

  }
  ;

  double interpolate(Eigen::MatrixXd dataY, Eigen::VectorXd dataX1, Eigen::VectorXd dataX2,
                     double value1, double value2)
  {
    if (value1 > dataX1[dataX1.size() - 1]) {
      std::cout << "Value1 : " << value1 << " is exceed max : " << dataX1[dataX1.size() - 1]
                << "!!\n";
      return -1000;
    }
    if (value1 < dataX1[0]) {
      std::cout << "Value1 : " << value1 << " is exceed min : " << dataX1[0] << "!!\n";
      return -1000;
    }
    if (value2 > dataX2[dataX2.size() - 1]) {
      std::cout << "Value2 : " << value2 << " is exceed max : " << dataX2[dataX2.size() - 1]
                << "!!\n";
      return -1000;
    }
    if (value2 < dataX2[0]) {
      std::cout << "Value2 : " << value2 << " is exceed min : " << dataX2[0] << "!!\n";
      return -1000;
    }

    // search indeces corresponding to correct section of X values
    int index1 = 0;
    int index2 = 0;
    while (index1 < dataX1.size()) {
      if (dataX1[index1] > value1) {
        break;
      }
      index1++;
    }
    while (index2 < dataX2.size()) {
      if (dataX2[index2] > value2) {
        break;
      }
      index2++;
    }
    Eigen::VectorXd col;
    if (index1 == dataX1.size()) {
      // return max value
      col = dataY.col(index1 - 1);
    } else {
      // return interpolation
      col = dataY.col(index1 - 1)
          + (dataY.col(index1) - dataY.col(index1 - 1)) * (value1 - dataX1[index1 - 1])
              / (dataX1[index1] - dataX1[index1 - 1]);
    }

    if (index2 == dataX2.size()) {
      return col[index2 - 1];
    } else {
      return col[index2 - 1]
          + (col[index2] - col[index2 - 1]) * (value2 - dataX2[index2 - 1])
              / (dataX2[index2] - dataX2[index2 - 1]);
    }

  }
  ;

  Propeller propellerParameterCalculate(double Throttle, double vt, double tilt_angle,
                                        double tilt_angle_rad, double alphar, double betar,
                                        double rho)
  {
    // TODO : calculate propeller parameters here
    return Propeller();
  }
  // CALLBACKS
  void AileronRightCommandsCallback(const std_msgs::Float64& msg)
  {
    //std::lock_guard<std::mutex> lock(this->mutex_);
    angleAileronRight_ = msg.data;
  }
  void AileronLeftCommandsCallback(const std_msgs::Float64& msg)
  {
    //std::lock_guard<std::mutex> lock(this->mutex_);
    angleAileronLeft_ = msg.data;
  }
  void ElevatorRightCommandsCallback(const std_msgs::Float64& msg)
  {
    //std::lock_guard<std::mutex> lock(this->mutex_);
    angleElevatorRight_ = msg.data;

  }
  void ElevatorLeftCommandsCallback(const std_msgs::Float64& msg)
  {
    //std::lock_guard<std::mutex> lock(this->mutex_);
    angleElevatorLeft_ = msg.data;
  }

 protected:

  // TODO Mehmet Efe Tiryaki 14.11.2018 : these parameters are hardcoded in constructor
  // bad programming, create config yaml/xml import in read parameters through rosparam
  double b; /* span, m */
  double b_wake;
  double S; /* planform area, m^2 */
  double cbar; /* mean aero chord, m */
  double cbar_wake; /* mean aero chord of the region after propeller, m */
  double Jy; /* kg-m^2 */
  double Jxz; /* kg-m^2 */
  double Jz; /* kg-m^2 */
  double Jx; /* kg-m^2 */

  Eigen::VectorXd ail_;
  Eigen::VectorXd alpha_wake_;
  Eigen::VectorXd alpha_;
  Eigen::VectorXd alphalim_wake_;
  Eigen::VectorXd alphalim_;
  Eigen::MatrixXd cd_alpha_elv_;
  Eigen::VectorXd cd_alpha_wake_;
  Eigen::VectorXd cd_alpha_;
  Eigen::VectorXd cl_ail_;
  Eigen::VectorXd cl_alpha_wake_;
  Eigen::VectorXd cl_alpha_;
  Eigen::VectorXd cl_beta_;
  Eigen::VectorXd cl_elv_;
  Eigen::VectorXd cl_p_;
  Eigen::VectorXd cl_r_;
  Eigen::VectorXd cm_alpha_wake_;
  Eigen::VectorXd cm_alpha_;
  Eigen::VectorXd cm_elv_;
  Eigen::MatrixXd cn_ail_;
  Eigen::VectorXd cn_beta_;
  Eigen::VectorXd cn_p_;
  Eigen::VectorXd cn_r_;
  Eigen::VectorXd cy_p_;
  Eigen::VectorXd ele_;
  Eigen::VectorXd elelim_;

  double CL_0;
  double CL_q;
  double CL_alpha_dot;
  double CL_elv;
  double CD_0;
  double CD_elv;
  double CM_0;
  double CM_q;
  double CM_ad;
  double CM_elv;
  double CY_beta;
  double CY_p;
  double CY_r;
  double CY_ail;
  double Cl_beta;
  double Cl_p;
  // TODO Mehmet Efe Tiryaki 14.11.2018 : some random exclamation w/o explanation
  double Cl_r;    //%%%%%%%%%%%% !!!!!!!!!!
  double Cl_ail;
  double Cl_rud;
  double CN_beta;
  // TODO Mehmet Efe Tiryaki 14.11.2018 : some random exclamation w/o explanation
  double CN_p;    //%%%%%%%%%%%% !!!!!!!!!!
  double CN_r;
  double CN_ail;
  double CN_rud;
  double CL_0_wake0;
  double CL_q_wake0;
  double CL_alpha_dot_wake0;
  double CD0_wake0;
  double CM_0_wake0;
  double CM_q_wake0;
  double CM_ad_wake0;
  double CY_beta_wake0;
  double CY_p_wake0;
  double CY_r_wake0;
  double Cl_beta_wake0;
  double Cl_p_wake0;
  double Cl_r_wake0;
  double CN_beta_wake0;
  double CN_p_wake0;
  double CN_r_wake0;
  double CL_0_wake;
  double CL_q_wake;
  double CL_alpha_dot_wake;
  double CD0_wake;
  double CM_0_wake;
  double CM_q_wake;
  double CM_ad_wake;
  double CY_beta_wake;
  double CY_p_wake;
  double CY_r_wake;
  double Cl_beta_wake;
  double Cl_p_wake;
  double Cl_r_wake;
  double CN_beta_wake;
  double CN_p_wake;
  double CN_r_wake;
  double CL_wake;
  double CD_wake;
  double CD_wake0;
  double CL_wake0;
  double CD0;

  Eigen::Vector3d B_wake;
  Eigen::Vector3d B;

  double CL;
  double CD;
  Eigen::Vector3d CF;
  Eigen::Vector3d CM;
  Eigen::Vector3d CF_wake0;
  Eigen::Vector3d CM_wake0;
  Eigen::Vector3d CF_wake;
  Eigen::Vector3d CM_wake;

  double mass_;
  double qbar_;
  double qbar_wake_;
  double rho_;
  double S_wake_;
  double velocity_;
  double velocityEffective_;
  // Angle of Attack
  double angleOfAttack_;
  double angleOfAttackDot_;
  double angleOfAttackEffective_;
  // Side Slip Angle
  double sideSlipAngle_;

  // Control surface angles
  double angleAileronRight_;
  double angleAileronLeft_;
  double angleElevatorRight_;
  double angleElevatorLeft_;

  // AERODYNAMIC FORCES and Moments
  Eigen::Vector3d F_free_;
  Eigen::Vector3d F_wake_;
  Eigen::Vector3d F_wake0_;
  Eigen::Vector3d M_free_;
  Eigen::Vector3d M_wake_;
  Eigen::Vector3d M_wake0_;

  // Subscriber
  ros::Subscriber aileronRightSubscriber_;
  ros::Subscriber aileronLeftSubscriber_;
  ros::Subscriber elevatorRightSubscriber_;
  ros::Subscriber elevatorLeftSubscriber_;
};

}  // namespace effort
}  // namespace gazebo
