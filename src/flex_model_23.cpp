#include <flex_model_23.h>


using namespace Eigen;
using namespace std;


void flexmod23::compute(const Matrix<double,7,1> &pos_ang,const double &charge)
{
	init_data(pos_ang, charge);
	/* stiffness parameter */
	double K[6] = {5.106E4 , 4.273E4 , 4.223E4 , 4.146E4 , 4.344E4 , 4.370E4};

	/* initialisation problem */
	/*double phi[6];
	double adj[6];
	Matrix3d R1;
	Matrix3d R2;
	Matrix3d R3;
	Matrix3d R4;
	Matrix3d R5;
	Matrix3d R6;
	Matrix3d R_int;
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

	adj[0] = -atan2( pos7(0)-pos1(1) , pos7(1)-pos1(1) );
	adj[1] = -atan2( pos7(0)-pos2(1) , pos7(1)-pos2(1) );
	adj[2] = -atan2( pos7(0)-pos3(1) , pos7(1)-pos3(1) );
	adj[3] = -atan2( pos7(0)-pos4(1) , pos7(1)-pos4(1) );
	adj[4] = -atan2( pos7(0)-pos5(1) , pos7(1)-pos5(1) );
	adj[5] = -atan2( pos7(0)-pos6(1) , pos7(1)-pos6(1) );
	

	/* deflection angle */
	for (int j = 0; j < 6; j++)
	{
		phi[j] = torque(j) / ( K[j] * 1000 );
	}


	/* rotation matrix */

	for (int j = 0 ; j<6 ; j++)
	{
		R_int << 1, 0, 0,
			0 , cos(phi[j]) , sin(phi[j]),
			0 , -sin(phi[j]) , cos(phi[j]);
			
		T_int << cos(adj[j]) , -sin(adj[j]) , 0,
			sin(adj[j]), cos(adj[j]), 0,
			0, 0, 1;

		R = T_int * R_int * T_int.transpose();
	
		switch (j) {
			case 0 : R1 = R;
				break;
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
