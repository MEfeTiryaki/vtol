/*
 File name: AerodynamicContainerBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 10.11.2018
 Date last modified: 10.11.2018
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

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Eigen
#include <Eigen/Dense>

class AerodynamicContainerBase
{
 public:
  AerodynamicContainerBase()
      : nodeHandle_()
  {
  }
  ;

  virtual ~AerodynamicContainerBase()
  {
  }
  ;

  virtual void initilize(ros::NodeHandle* nodeHandle)
  {
    nodeHandle_ = nodeHandle;

    ns_ = ros::this_node::getNamespace();
  }

  virtual void readParameters()
  {
  }
  ;

  virtual void updateAerodyamics()
  {
  }
  ;
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

 protected:

  ros::NodeHandle* nodeHandle_;

  std::string ns_;
};
