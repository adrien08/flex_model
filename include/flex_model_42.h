#ifndef __FLEX_MODEL42_HPP__
#define __FLEX_MODEL42_HPP__

#include <flex_model.h>

class flexmod42: public FlexModel{

public:
  flexmod42(){
  }
  Eigen::Matrix<double, 7 , 1> compute(const Eigen::Matrix<double,7,1> &pos_ang,const double &charge);


  
};

#endif
