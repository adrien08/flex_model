#ifndef __FLEX_MODEL_HPP__
#define __FLEX_MODEL_HPP__

#include <iostream>
#include <Eigen/Dense>

class FlexModel{
public:
  FlexModel(){}
  // Methodes
  virtual void compute(const Eigen::Matrix<double,7,1> &pos_ang,const double &charge,Eigen::Matrix<double,7,1> &correction_pos_ang)=0;
  
  double norm2(const Eigen::Vector3d &x);
  
  void init_data(const Eigen::Matrix<double,7,1>& pos_ang ,const double charge);
  
  Eigen::Matrix<double,7,1> rotation_model(Eigen::Vector3d &pos1,
	       Eigen::Vector3d &pos2,
	       Eigen::Vector3d &pos3,
	       Eigen::Vector3d &pos4,
	       Eigen::Vector3d &pos5,
	       Eigen::Vector3d &pos6,
	       Eigen::Vector3d &pos7,
	       const Eigen::Matrix3d &R1,
	       const Eigen::Matrix3d &R2,
	       const Eigen::Matrix3d &R3,
	       const Eigen::Matrix3d &R4,
	       const Eigen::Matrix3d &R5,
	       const Eigen::Matrix3d &R6,
	       Eigen::Vector3d &Model);
  
   void dh_param(const Eigen::Matrix<double,7,1>& pos_ang ,
		const double l[7],
		Eigen::Matrix<double,24,4> &transformation_matrix);
   
   Eigen::Vector4d quaternionFromMatrix(const Eigen::Matrix3d &M);
  
 // Atributs
	Eigen::Matrix<double, 24 , 4> transformation_matrix;
	Eigen::Matrix<double,6,1> torque;
	Eigen::Vector3d joint_position1;
	Eigen::Vector3d joint_position2;
	Eigen::Vector3d joint_position3;
	Eigen::Vector3d joint_position4;
	Eigen::Vector3d joint_position5;
	Eigen::Vector3d joint_position6;
	Eigen::Vector3d joint_position7;
	Eigen::Matrix<double,7,1> mod_12;
	Eigen::Matrix<double,7,1> mod_23;
	double adj[6];
	Eigen::Matrix<double,7,1> mod_42;
	double pi;
	double F;
	double l[7];
	double l_bar3;
	double l_bar2;
	double l_bar4;
	double l_bar5;
	double phi[6];
	double coeff[6];
	Eigen::Matrix3d R_int;
	double phi_x[6];
	double phi_y[6];
	double phi_axe[6];
	double coeff_x[6];
	double coeff_y[6];
	double coeff_axe[6];
	Eigen::Matrix<double,1,6> tau;
	Eigen::Matrix3d R1_x;
	Eigen::Matrix3d R_x;
	Eigen::Matrix3d R1_y;
	Eigen::Matrix3d R_y;
	Eigen::Matrix3d R1_axe;
	Eigen::Matrix3d R_axe;
	Eigen::Matrix3d R1;
	Eigen::Matrix3d R2;
	Eigen::Matrix3d R3;
	Eigen::Matrix3d R4;
	Eigen::Matrix3d R5;
	Eigen::Matrix3d R6;
	Eigen::Matrix3d T_int;
	Eigen::Matrix3d R;
	Eigen::Vector3d Model;
	Eigen::Matrix3d Model_orientation;
	Eigen::Vector4d quat_corr;
	Eigen::Vector3d corr_model;
	Eigen::Matrix<double,7,1> corr_trans_rot;
  
};


#endif