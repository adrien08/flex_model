#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <lwr_flex_models/flex_model_12.h>
#include <lwr_flex_models/flex_model_23.h>
#include <lwr_flex_models/flex_model_42.h>
#include <sys/time.h>

using namespace Eigen;
using namespace std;

int main()
{
	// Variables timer
	struct timeval tbegin,tend;
	double texec=0.;

	// Param
	const double pi = 3.14159265359;
	Matrix<double,7,1> pos_ang;
	pos_ang << 0,90,0,0,0,0,0;
	pos_ang = (pi / 180) * pos_ang;
	Matrix<double,6,1> effort;
	effort << 0,50,-0,0,0,0;
	
	// definition of the correction 

	flexmod12 mod12;
	flexmod42 mod42;
	
	Eigen::Matrix<double,7,1> out12,out23,out42;
	
	// Start timer
	gettimeofday(&tbegin,NULL);

	mod12.compute(pos_ang, effort ,out12);
	mod42.compute(pos_ang, effort ,out42);

	// End timer
	gettimeofday(&tend,NULL);
	
	
	// print to screen

	
	cout << "La position cartésienne rigide actuelle est" << endl;
	cout << mod42.joint_position7 << endl;

	cout << "Les résultats suivant sont donnés sous la forme : " << endl;
	cout << "Correction à approter en translation : " << endl;
	cout << "x" << endl;
	cout << "y" << endl;
	cout << "z" << endl;
	cout << "Correction à approter en rotation (Quaternion) : " << endl;
	cout << "x" << endl;
	cout << "y" << endl;
	cout << "z" << endl;
	cout << "w" << endl;

	cout << "La correction à apporter selon le modèle 12 est " << endl;
	cout << out12 << endl;

	cout << "La correction à apporter selon le modèle 42 est " << endl;
	cout << out42 << endl;
	
            

	
	// Compute execution time
	texec= 1000*(tend.tv_sec-tbegin.tv_sec)+(tend.tv_usec-tbegin.tv_usec)/1000.;

	cout << "Execution time : " << texec << "ms" << endl ;


	return 0;
}

