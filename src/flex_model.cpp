#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <lwr_flex_models/flex_model.h>

using namespace Eigen;
using namespace std;

void FlexModel::init_data(const Matrix<double,7,1>& pos_ang ,const double charge)
{
	// param 	

	pi = 3.14159265359;

	// parameters of the experience

	F = 9.81 * charge;

	// length of the arms
	for (int i ; i<7 ; i++){
	l[0] = 110.0;
	l[1] = 200.0;
	l[2] = 200.0;
	l[3] = 200.0;
	l[4] = 200.0;
	l[5] = 190.0;
	l[6] = 133.1547;
	l_bar3 = l[3]/(l[2]+l[3]);
	l_bar2 = l[2]/(l[2]+l[3]);
	l_bar5 = l[5]/(l[4]+l[5]);
	l_bar4 = l[4]/(l[4]+l[5]);
	}

	// launch DH parameters

	dh_param(pos_ang , l, transformation_matrix);

	// position from dh parameters

	joint_position1 << 0 , 0 , l[0];

	joint_position2 << transformation_matrix(4,3) , transformation_matrix(5,3) , transformation_matrix(6,3);

	joint_position3 << transformation_matrix(4,3)*l_bar3 + transformation_matrix(12,3)*l_bar2 , transformation_matrix(5,3)*l_bar3 + transformation_matrix(13,3)*l_bar2 , transformation_matrix(6,3)*l_bar3 + transformation_matrix(14,3)*l_bar2 ;

	joint_position4 << transformation_matrix(12,3) , transformation_matrix(13,3) , transformation_matrix(14,3);

	joint_position5 << transformation_matrix(12,3)*l_bar5 + transformation_matrix(16,3)*l_bar4  , transformation_matrix(13,3)*l_bar5 + transformation_matrix(17,3)*l_bar4  , transformation_matrix(14,3)*l_bar5 + transformation_matrix(18,3)*l_bar4 ;

	joint_position6 << transformation_matrix(16,3) , transformation_matrix(17,3) , transformation_matrix(18,3);

	joint_position7 << transformation_matrix(20,3) , transformation_matrix(21,3) , transformation_matrix(22,3); // 3d effector position

	// torque (general expression)

	torque(0) = F * norm2( joint_position7 - joint_position1 );
	torque(1) = F * norm2( joint_position7 - joint_position2 );
	torque(2) = F * norm2( joint_position7 - joint_position3 );
	torque(3) = F * norm2( joint_position7 - joint_position4 );
	torque(4) = F * norm2( joint_position7 - joint_position5 );
	torque(5) = F * norm2( joint_position7 - joint_position6 );
}

double FlexModel::norm2(const Vector3d &x)
{
	return sqrt(x(0)*x(0) + x(1)*x(1));	
}

Vector4d FlexModel::quaternionFromMatrix( const Matrix3d &M)
{
	Vector4d quat;
	double fourWSquaredMinus1=M(0,0)+M(1,1)+M(2,2);
	double fourXSquaredMinus1=M(0,0)-M(1,1)-M(2,2);
	double fourYSquaredMinus1=M(1,1)-M(0,0)-M(2,2);
	double fourZSquaredMinus1=M(2,2)-M(0,0)-M(1,1);

	int biggestIndex=0;
	double fourBiggestSquredMinus1=fourWSquaredMinus1;
	if(fourXSquaredMinus1>fourBiggestSquredMinus1){
	fourBiggestSquredMinus1=fourXSquaredMinus1;
	biggestIndex=1;
	}
	if(fourYSquaredMinus1>fourBiggestSquredMinus1){
	fourBiggestSquredMinus1=fourYSquaredMinus1;
	biggestIndex=2;
	}
	if(fourZSquaredMinus1>fourBiggestSquredMinus1){
	fourBiggestSquredMinus1=fourZSquaredMinus1;
	biggestIndex=3;
	}

	double biggestVal=sqrt(fourBiggestSquredMinus1+1.0f)*0.5f;
	double mult=0.25f/biggestVal;

	switch(biggestIndex)
	{
	case 0:
		quat <<
	    ((M(1,2)-M(2,1))*mult),
	    ((M(2,0)-M(0,2))*mult),
	    ((M(0,1)-M(1,0))*mult),
	    (biggestVal);
		break;
	case 1:
		quat <<
	    (biggestVal),
	    ((M(0,1)+M(1,0))*mult),
	    ((M(2,0)+M(0,2))*mult),
	    ((M(1,2)-M(2,1))*mult);
		break;
	case 2:
		quat <<
	    ((M(0,1)+M(1,0))*mult),
	    (biggestVal),
	    ((M(1,2)+M(2,1))*mult),
	    ((M(2,0)-M(0,2))*mult);
		break;
	case 3:
	quat <<
	    ((M(2,0)+M(0,2))*mult),
	    ((M(1,2)+M(2,1))*mult),
	    (biggestVal),
	    ((M(0,1)-M(1,0))*mult);
		break;
	default:
		assert(false);
	return quat;
	}
}

void FlexModel::dh_param(const Matrix<double,7,1>& pos_ang ,
		const double l[7] ,
		Matrix<double,24,4> &transformation_matrix)
{
	Matrix4d C;
	Matrix4d B;
	Matrix<double, 6 , 4> F;
	double pi = 3.14159265359;
	/* dh parameters */
	
	double a1 = 0;
	double d1 = l[0] + l[1];
	double alpha1 = pi/2;

	double a2 = 0;
	double d2 = 0;
	double alpha2 = pi/2;

	double a3 = 0;
	double d3 = l[2] + l[3];
	double alpha3 = pi/2;

	double a4 = 0;
	double d4 = 0;
	double alpha4 = pi/2;

	double a5 = 0;
	double d5 = l[4] + l[5];
	double alpha5 = pi/2;

	double a6 = l[6];
	double d6 = 0;
	double alpha6 = 0;

	/* transformation matrix */

	F << a1, alpha1, d1, pos_ang[0],
	a2, alpha2, d2,	pos_ang[1] - pi,
	a3, alpha3, d3,	pos_ang[2],
	a4, alpha4, d4,	pos_ang[3] + pi,
	a5, alpha5, d5,	pos_ang[4],
	a6, alpha6, d6,	pos_ang[5] + pi/2;

	B << 1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1;

	for(int i = 0; i < 6 ; i++)
	{
		C << cos(F(i,3)), -sin(F(i,3))*cos(F(i,1)), sin(F(i,3))*sin(F(i,1)), F(i,0)*cos(F(i,3)),
			sin(F(i,3)), cos(F(i,3))*cos(F(i,1)), -cos(F(i,3))*sin(F(i,1)), F(i,0)*sin(F(i,3)),
			0, sin(F(i,1)), cos(F(i,1)), F(i,2),
			0, 0, 0, 1;
		B = B * C;
		for (int m = 0 ; m < 4 ; m++) {
		    for (int j = 0 ; j < 4 ; j++) {
			transformation_matrix(4*(i) + m , j) = B(m,j);
			}
		    }
	}
}

Matrix<double, 7 , 1> FlexModel::rotation_model(Vector3d &pos1,
           Vector3d &pos2,
           Vector3d &pos3,
           Vector3d &pos4,
           Vector3d &pos5,
           Vector3d &pos6,
           Vector3d &pos7,
           const Matrix3d &R1,
           const Matrix3d &R2,
           const Matrix3d &R3,
           const Matrix3d &R4,
           const Matrix3d &R5,
           const Matrix3d &R6,
           Vector3d &Model)
{

	Model = pos7;

	// for joint positions
	pos2 = R1*(pos2 - pos1) + pos1;
	pos3 = R1*(pos3 - pos1) + pos1;
	pos4 = R1*(pos4 - pos1) + pos1;
	pos5 = R1*(pos5 - pos1) + pos1;
	pos6 = R1*(pos6 - pos1) + pos1;

	pos3 = R2*(pos3 - pos2) + pos2;
	pos4 = R2*(pos4 - pos2) + pos2;
	pos5 = R2*(pos5 - pos2) + pos2;
	pos6 = R2*(pos6 - pos2) + pos2;

	pos4 = R3*(pos4 - pos3) + pos3;
	pos5 = R3*(pos5 - pos3) + pos3;
	pos6 = R3*(pos6 - pos3) + pos3;

	pos5 = R4*(pos5 - pos4) + pos4;
	pos6 = R4*(pos6 - pos4) + pos4;

	pos6 = R5*(pos6 - pos5) + pos5;


	// for model positions
	Model = R1*(Model - pos1) + pos1;
	Model = R2*(Model - pos2) + pos2;
	Model = R3*(Model - pos3) + pos3;
	Model = R4*(Model - pos4) + pos4;
	Model = R5*(Model - pos5) + pos5;
	Model = R6*(Model - pos6) + pos6;

	// for model orientation
	
	Model_orientation = R6*R5*R4*R3*R2*R1;
	quat_corr = quaternionFromMatrix(Model_orientation.transpose());
	corr_model = pos7 - Model;

	corr_trans_rot << corr_model(0), corr_model(1), corr_model(2), quat_corr(0), quat_corr(1), quat_corr(2), quat_corr(3);
	return corr_trans_rot;
}
