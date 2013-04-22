#ifndef PREVIEWCONTROLLER_H
#define PREVIEWCONTROLLER_H

#include <Eigen/Core>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

class PreviewController
{
private:
	int m_nPreviewNum;		// number of ZMP points to look ahead
	double m_dg;			// gravity acceleration
	double m_dG1;			// accumulated ZMP error gain
	Vector3d m_vG2;			// state feedback gain
	VectorXd m_vG3;			// preview gain
	double m_dETotal;		// accumulated ZMP error
	double m_dETolerance;	// error tolerance when computing gains

	Matrix3d m_mA;
	Vector3d m_vB;
	RowVector3d m_vC; 

	Vector3d m_vX;
	double m_dZMPXEXP;

	Vector3d m_vY;
	double m_dZMPYEXP;

	double m_du;
	x{1} = [ 0; 0; 0];
y(1) = 0;
yexp(1) = 0;
public:
	PreviewController(double dT, double dZc, 
						double dQe, double dQu, 
						int nN = 320, double dg = 9.81, double dETolerance = 1e-10);
	void PrintSaveGains();
};

#endif