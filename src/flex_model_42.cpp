#include <lwr_flex_models/flex_model_42.h>

using namespace Eigen;
using namespace std;


void flexmod42::compute(const Matrix<double,7,1> &pos_ang,const double &charge)
{
	init_data(pos_ang, charge);
	/* stiffness parameter */
	double K[6][2] = 
	{
		{2.726E4 , 7.328E4},
		{2.893E4 , 7.169E4},
		{2.802E4 , 7.153E4},
		{1.963E4 , 7.255E4},
		{2.705E4 , 7.228E4},
		{2.698E4 , 7.241E4}
	};
	
	/* initialisation problem */
	/*double phi_x[6];
	double phi_y[6];
	double phi_axe[6];
	double coeff_x[6];
	double coeff_y[6];
	double coeff_axe[6];
	Matrix<double,1,6> tau;
	Matrix3d R1_x;
	Matrix3d R_x;
	Matrix3d R1_y;
	Matrix3d R_y;
	Matrix3d R1_axe;
	Matrix3d R_axe;
	Matrix3d R1;
	Matrix3d R2;
	Matrix3d R3;
	Matrix3d R4;
	Matrix3d R5;
	Matrix3d R6;
	Matrix3d T_int;
	Matrix3d R;
	Vector3d Model;
	Matrix3d Model_orientation;
	Vector4d quat_corr;
	Vector3d corr_model;
	Matrix<double,7,1> corr_trans_rot;*/

	Vector3d pos1;
	pos1 = joint_position1;
	Vector3d pos2;
	pos2 = joint_position2;
	Vector3d pos3;
	pos3 = joint_position3;
	Vector3d pos4;
	pos4 = joint_position4;
	Vector3d pos5;
	pos5 = joint_position5;
	Vector3d pos6;
	pos6 = joint_position6;
	Vector3d pos7;
	pos7 = joint_position7;

	/* axis orientation */
	Vector3d t1_axe(0,0,1);
	Vector3d t2_axe(transformation_matrix(0,2),transformation_matrix(1,2),transformation_matrix(2,2));
	Vector3d t3_axe(transformation_matrix(4,2),transformation_matrix(5,2),transformation_matrix(6,2));
	Vector3d t4_axe(transformation_matrix(8,2),transformation_matrix(9,2),transformation_matrix(10,2));
	Vector3d t5_axe(transformation_matrix(12,2),transformation_matrix(13,2),transformation_matrix(14,2));
	Vector3d t6_axe(transformation_matrix(16,2),transformation_matrix(17,2),transformation_matrix(18,2));

	Vector3d t1_x(1,0,0);
	Vector3d t2_x(transformation_matrix(0,0),transformation_matrix(1,0),transformation_matrix(2,0));
	Vector3d t3_x(transformation_matrix(4,0),transformation_matrix(5,0),transformation_matrix(6,0));
	Vector3d t4_x(transformation_matrix(8,0),transformation_matrix(9,0),transformation_matrix(10,0));
	Vector3d t5_x(transformation_matrix(12,0),transformation_matrix(13,0),transformation_matrix(14,0));
	Vector3d t6_x(transformation_matrix(16,0),transformation_matrix(17,0),transformation_matrix(18,0));

	Vector3d t1_y(0,1,0);
	Vector3d t2_y(transformation_matrix(0,1),transformation_matrix(1,1),transformation_matrix(2,1));
	Vector3d t3_y(transformation_matrix(4,1),transformation_matrix(5,1),transformation_matrix(6,1));
	Vector3d t4_y(transformation_matrix(8,1),transformation_matrix(9,1),transformation_matrix(10,1));
	Vector3d t5_y(transformation_matrix(12,1),transformation_matrix(13,1),transformation_matrix(14,1));
	Vector3d t6_y(transformation_matrix(16,1),transformation_matrix(17,1),transformation_matrix(18,1));


	/* correction coefficient */
	coeff_axe[0] = ( t1_axe .cross( t1_axe)) .dot( pos7 - pos1 ) / norm2(pos7 - pos1);
	coeff_axe[1] = ( t1_axe .cross( t2_axe)) .dot( pos7 - pos2 ) / norm2(pos7 - pos2);
	coeff_axe[2] = ( t1_axe .cross( t3_axe)) .dot( pos7 - pos3 ) / norm2(pos7 - pos3);
	coeff_axe[3] = ( t1_axe .cross( t4_axe)) .dot( pos7 - pos4 ) / norm2(pos7 - pos4);
	coeff_axe[4] = ( t1_axe .cross( t5_axe)) .dot( pos7 - pos5 ) / norm2(pos7 - pos5);
	coeff_axe[5] = ( t1_axe .cross( t6_axe)) .dot( pos7 - pos6 ) / norm2(pos7 - pos6);

	coeff_x[0] = ( t1_axe .cross( t1_x)) .dot( pos7 - pos1 ) / norm2(pos7 - pos1);
	coeff_x[1] = ( t1_axe .cross( t2_x)) .dot( pos7 - pos2 ) / norm2(pos7 - pos2);
	coeff_x[2] = ( t1_axe .cross( t3_x)) .dot( pos7 - pos3 ) / norm2(pos7 - pos3);
	coeff_x[3] = ( t1_axe .cross( t4_x)) .dot( pos7 - pos4 ) / norm2(pos7 - pos4);
	coeff_x[4] = ( t1_axe .cross( t5_x)) .dot( pos7 - pos5 ) / norm2(pos7 - pos5);
	coeff_x[5] = ( t1_axe .cross( t6_x)) .dot( pos7 - pos6 ) / norm2(pos7 - pos6);

	coeff_y[0] = ( t1_axe .cross( t1_y)) .dot( pos7 - pos1 ) / norm2(pos7 - pos1);
	coeff_y[1] = ( t1_axe .cross( t2_y)) .dot( pos7 - pos2 ) / norm2(pos7 - pos2);
	coeff_y[2] = ( t1_axe .cross( t3_y)) .dot( pos7 - pos3 ) / norm2(pos7 - pos3);
	coeff_y[3] = ( t1_axe .cross( t4_y)) .dot( pos7 - pos4 ) / norm2(pos7 - pos4);
	coeff_y[4] = ( t1_axe .cross( t5_y)) .dot( pos7 - pos5 ) / norm2(pos7 - pos5);
	coeff_y[5] = ( t1_axe .cross( t6_y)) .dot( pos7 - pos6 ) / norm2(pos7 - pos6);
	


	/* deflection angle */
	tau = torque;
	for (int j = 0; j < 6; j++)
	{
		phi_x[j] = coeff_x[j] * tau(j) / ( K[j][1] * 1000 );
		phi_y[j] = coeff_y[j] * tau(j) / ( K[j][1] * 1000 );
		phi_axe[j] = coeff_axe[j] * tau(j) / ( K[j][0] * 1000 );
	}


	/* rotation matrix */


	R1_x << 1,0,0, 0,cos(phi_x[0]),sin(phi_x[0]), 0,-sin(phi_x[0]),cos(phi_x[0]);
	R1_y << cos(phi_y[0]),0,-sin(phi_y[0]), 0,1,0, sin(phi_y[0]),0,cos(phi_y[0]);
	R1_axe << 1,0,0, 0,1,0, 0,0,1;
	R1 = R1_x * R1_y * R1_axe;

	for (int j = 1 ; j<6 ; j++)
	{
		R_axe << cos(phi_axe[j]),sin(phi_axe[j]),0, -sin(phi_axe[j]), cos(phi_axe[j]),0, 0,0,1;
		R_x << 1,0,0, 0,cos(phi_x[j]),sin(phi_x[j]), 0,-sin(phi_x[j]),cos(phi_x[j]);
		R_y << cos(phi_y[j]),0,-sin(phi_y[j]), 0,1,0, sin(phi_y[j]),0,cos(phi_y[j]);

		T_int << transformation_matrix(4*(j-1),0), transformation_matrix(4*(j-1),1), transformation_matrix(4*(j-1),2),
			transformation_matrix(4*(j-1)+1,0), transformation_matrix(4*(j-1)+1,1), transformation_matrix(4*(j-1)+1,2),
			transformation_matrix(4*(j-1)+2,0), transformation_matrix(4*(j-1)+2,1), transformation_matrix(4*(j-1)+2,2);

		R = T_int * R_x * R_y * R_axe * T_int.transpose();
	
		switch (j) {
			case 1 : R2 = R;
				break;
			case 2 : R3 = R;
				break;
			case 3 : R4 = R;
				break;
			case 4 : R5 = R;
				break;
			case 5 :  R6 = R;
				break;
		}
	}	
	corr_trans_rot = FlexModel::rotation_model(joint_position1,
	       joint_position2,
	       joint_position3,
	       joint_position4,
	       joint_position5,
	       joint_position6,
	       joint_position7,
	       R1,
	       R2,
	      R3,
	       R4,
	       R5,
	       R6,
	       Model);
	return ;
}
