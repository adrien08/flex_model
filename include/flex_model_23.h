#ifndef __FLEX_MODEL23_HPP__
#define __FLEX_MODEL23_HPP__

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <flex_model.h>

class flexmod23: public FlexModel{

public:
  flexmod23(){
  }
  Eigen::Matrix<double, 7 , 1> compute(const Eigen::Matrix<double,7,1> &pos_ang,const double &charge);

  
};


#endif