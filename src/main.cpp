#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <flex_model.h>
#include <flex_model_12.h>
#include <flex_model_23.h>
#include <flex_model_42.h>
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
	double charge = 5;
	
	// definition of the correction 

	flexmod12 mod12;
	flexmod23 mod23;
	flexmod42 mod42;
	
	
	// Start timer
	gettimeofday(&tbegin,NULL);
	mod12.compute(pos_ang, charge);

	mod23.compute(pos_ang, charge);
	
	mod42.compute(pos_ang, charge);
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
	cout << mod12.corr_trans_rot << endl;

	cout << "La correction à apporter selon le modèle 23 est " << endl;
	cout << mod23.corr_trans_rot << endl;

	cout << "La correction à apporter selon le modèle 42 est " << endl;
	cout << mod42.corr_trans_rot << endl;
	
            

	
	// Compute execution time
	texec= 1000*(tend.tv_sec-tbegin.tv_sec)+(tend.tv_usec-tbegin.tv_usec)/1000.;

	cout << "Execution time : " << texec << "ms" << endl ;


	return 0;
}

