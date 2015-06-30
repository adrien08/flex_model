#ifndef __FLEX_MODEL12_HPP__
#define __FLEX_MODEL12_HPP__

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <flex_model.h>



class flexmod12: public FlexModel{

public:
  flexmod12(){
  }
  Eigen::Matrix<double, 7 , 1> compute(const Eigen::Matrix<double,7,1> &pos_ang,const double &charge);
  
};




#endif