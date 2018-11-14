/*
 File name: VtolAerodynamicContainer.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 10.11.2018
 Date last modified: 14.11.2018
 */

#include "vtol_gazebo/AerodynamicContainerBase.hpp"

class VtolAerodynamicContainer : public AerodynamicContainerBase
{
 public:
  VtolAerodynamicContainer()
      : AerodynamicContainerBase(),
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
        CD(0)
  {
  }
  ;

  virtual void readParameters()
  {
    std::vector<double> dummy;
    int row;
    int col;
    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/ail")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/ail", dummy);
      ail_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        ail_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The ail couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alpha_wake")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alpha_wake", dummy);
      alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        alpha_wake_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The alpha_wake couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alpha")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alpha", dummy);
      alpha_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        alpha_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The alpha couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim_wake")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim_wake", dummy);
      alphalim_wake_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        alphalim_wake_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The alphalim_wake couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/alphalim", dummy);
      alphalim_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        alphalim_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The alphalim couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv_row_number",
                                  row);
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_elv", dummy);
      col = dummy.size() / row;
      cd_alpha_elv_ = Eigen::MatrixXd::Zero(row, col);
      for (int i = 0; i < dummy.size(); i++) {
        cd_alpha_elv_(i / col, i % col) = dummy[i];
      }
    } else {
      ROS_INFO("The cd_alpha_elv_ couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_wake")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha_wake", dummy);
      cd_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cd_alpha_wake_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cd_alpha_wake couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cd_alpha", dummy);
      cd_alpha_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cd_alpha_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cd_alpha couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_ail")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_ail", dummy);
      cl_ail_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_ail_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_ail couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha_wake")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha_wake", dummy);
      cl_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_alpha_wake_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_alpha_wake couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_alpha", dummy);
      cl_alpha_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_alpha_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_alpha couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_beta")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_beta", dummy);
      cl_beta_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_beta_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_beta couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_elv")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_elv", dummy);
      cl_elv_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_elv_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_elv couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_p")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_p", dummy);
      cl_p_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_p_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_p couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cl_r")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cl_r", dummy);
      cl_r_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cl_r_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cl_r couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha_wake")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha_wake", dummy);
      cm_alpha_wake_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cm_alpha_wake_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cm_alpha_wake couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_alpha", dummy);
      cm_alpha_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cm_alpha_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cm_alpha couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cm_elv")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cm_elv", dummy);
      cm_elv_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cm_elv_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cm_elv couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail_row_number", row);
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_ail", dummy);
      col = dummy.size() / row;
      cn_ail_ = Eigen::MatrixXd::Zero(row, col);
      for (int i = 0; i < dummy.size(); i++) {
        cn_ail_(i / col, i % col) = dummy[i];
      }
    } else {
      ROS_INFO("The cn_ail couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_beta")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_beta", dummy);
      cn_beta_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cn_beta_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cn_beta couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_p")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_p", dummy);
      cn_p_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cn_p_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cn_p couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cn_r")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cn_r", dummy);
      cn_r_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cn_r_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cn_r couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/cy_p")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/cy_p", dummy);
      cy_p_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        cy_p_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The cy_p couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/ele")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/ele", dummy);
      ele_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        ele_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The ele couldn't be found");
    }

    if (this->nodeHandle_->hasParam(this->ns_ + "/vtol_0001/aerodynamics/elelim")) {
      this->nodeHandle_->getParam(this->ns_ + "/vtol_0001/aerodynamics/elelim", dummy);
      elelim_ = Eigen::VectorXd::Zero(dummy.size());
      for (int i = 0; i < dummy.size(); i++) {
        elelim_(i) = dummy[i];
      }
    } else {
      ROS_INFO("The elelim couldn't be found");
    }
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

  Eigen::Vector3d getB()
  {
    return B;
  }
  ;

  Eigen::Vector3d getBWake()
  {
    return B_wake;
  }

  Eigen::Vector3d getCFWake0()
  {
    return CF_wake0;
  }
  ;

  Eigen::Vector3d getCMWake0()
  {
    return CM_wake0;
  }
  ;
  Eigen::Vector3d getCFWake()
  {
    return CF_wake;
  }
  ;
  Eigen::Vector3d getCMWake()
  {
    return CM_wake;
  }
  ;
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

};
