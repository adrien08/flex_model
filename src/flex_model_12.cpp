#include <lwr_flex_models/flex_model_12.h>


using namespace Eigen;
using namespace std;

void flexmod12::compute(const Matrix<double,7,1> &pos_ang,const Eigen::Matrix<double,6,1> & effort,Matrix<double,7,1> &correction_pos_ang)
{
	init_data(pos_ang, effort);
	/* stiffness parameter */
	double K[6] = {1.726E4 , 1.735E4 , 1.725E4 , 1.706E4 , 1.726E4 , 1.719E4};

	/* axis orientation */
	
	Vector3d t1(0,0,1);
	Vector3d t2(transformation_matrix(0,2),transformation_matrix(1,2),transformation_matrix(2,2));
	Vector3d t3(transformation_matrix(4,2),transformation_matrix(5,2),transformation_matrix(6,2));
	Vector3d t4(transformation_matrix(8,2),transformation_matrix(9,2),transformation_matrix(10,2));
	Vector3d t5(transformation_matrix(12,2),transformation_matrix(13,2),transformation_matrix(14,2));
	Vector3d t6(transformation_matrix(16,2),transformation_matrix(17,2),transformation_matrix(18,2));


	/* correction coefficient */
	Matrix<double,6,1> matrix0;
	matrix0 << 0,0,0,0,0,0;
	if (torque != matrix0)
	
	coeff[0] = -torque_0 .dot( t1);
	coeff[1] = -torque_1 .dot( t2);
	coeff[2] = -torque_2 .dot( t3);
	coeff[3] = -torque_3 .dot( t4);
	coeff[4] = -torque_4 .dot( t5);
	coeff[5] = -torque_5 .dot( t6);
	
	/* deflection angle */
	tau = torque;
	for (int j = 0; j < 6; j++)
	{
		tau(j) = coeff[j] * tau(j);
		phi[j] = tau(j) / ( K[j] * 1000 );
	}

	/* rotation matrix */
	
	R1 << cos(phi[0]) , sin(phi[0]) , 0,
	      -sin(phi[0]), cos(phi[0]), 0,
	      0, 0, 1;
	
	for (int j = 1 ; j<6 ; j++)
	{
		
		R_int << cos(phi[j]) , sin(phi[j]) , 0,
			-sin(phi[j]), cos(phi[j]), 0,
			0, 0, 1;
		
		T_int << transformation_matrix(4*(j-1),0), transformation_matrix(4*(j-1),1), transformation_matrix(4*(j-1),2),
			transformation_matrix(4*(j-1)+1,0), transformation_matrix(4*(j-1)+1,1), transformation_matrix(4*(j-1)+1,2),
			transformation_matrix(4*(j-1)+2,0), transformation_matrix(4*(j-1)+2,1), transformation_matrix(4*(j-1)+2,2);

		
		R = T_int * R_int * T_int.transpose();
	
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
	correction_pos_ang = rotation_model(joint_position1,
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
