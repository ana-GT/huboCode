// (TODO)paper based
// learn form Matt's code

#include "PreviewController.h"
PreviewController::PreviewController(double dT, double dZc, 
						double dQe, double dQu, 
						int nN , double dg, double dETolerance)
	:m_nPreviewNum(nN), m_dg(dg), m_dETotal(0), m_dETolerance(dETolerance)
{

	m_vG3.resize(m_nPreviewNum + 1);	// plus one for the unused index 0 element

	Matrix3d mA;
	mA << 	1, dT, (dT*dT)/2, 
			0, 1, dT, 
			0, 0, 1;
	m_mA = mA;

	Vector3d vB;
	vB << (dT*dT*dT)/6, (dT*dT)/2, dT;
	m_vB = vB;

	RowVector3d vC; 
	vC << 1, 0, -dZc/dg;
	m_vC = vC;

	Vector4d vBB;
	vBB << vC*vB, vB;

	Vector4d vII;
	vII << 1, 0, 0, 0;

	Matrix<double, 4, 3> mFF;
	mFF << vC*mA, mA;

	Matrix4d mQQ;
	mQQ.setZero();
	mQQ(0,0) = dQe;

	Matrix4d mAA;
	mAA.topLeftCorner(4, 1) << vII;
	mAA.topRightCorner(4, 3) << mFF;

	Matrix4d mX;
	mX.setIdentity();

	Matrix4d mXnew;
	mXnew.setIdentity();

	double dCurrError;
	Matrix4d mATX;

	/*
	cout << "BB: \n" << vBB << endl;
	cout << "II: \n" << vII << endl;
	cout << "FF: \n" << mFF << endl;
	cout << "QQ: \n" << mQQ << endl;
	cout << "AA: \n" << mAA << endl;
	*/

	for (int i = 0; i < 10000; i++) {

		mATX = mAA.transpose() * mX;
		mXnew = mATX * mAA - mATX * vBB 
				* (1.0/(dQu + vBB.transpose() * mX * vBB))
				* vBB.transpose() * mATX.transpose()
				+ mQQ;

		dCurrError = (mXnew - mX).norm() / mXnew.norm();
		mX = mXnew;
		if (dCurrError < m_dETolerance) {
			cout << "PreviewController gains computation converge after "
				<< i+1 << " iterations" << endl;
			break;
		}

	}


	RowVector4d vW;
	RowVector4d vG;

	vW = 1.0/(dQu + vBB.transpose() * mX * vBB) * vBB.transpose();
	vG = vW * mX * mAA;
	m_dG1 = vG(0);
	m_vG2 = vG.tail(3);

	Matrix4d mTMP;
	Matrix4d mTMP2;

	mTMP = (mAA - vBB*vW*mX*mAA).transpose();
	mTMP2 = mTMP;
	m_vG3(0) = 0;
	m_vG3(1) = -m_dG1;

	for (int i = 2; i <= m_nPreviewNum; i++) {
		m_vG3(i) = -vW * mTMP2 * mX * vII; 
		mTMP2 *= mTMP;
	}
}

void PreviewController::PrintSaveGains()
{
	ofstream myfile;

	myfile.open("G3.txt");

	cout << "G1: " << m_dG1 << endl;
	cout << "G2: " << m_vG2 << endl;
	cout  << "G3: " << m_vG3 << endl;

	myfile << -m_vG3 << '\n';
	myfile.close();
}








