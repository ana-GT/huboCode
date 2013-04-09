/**
 * @file previewZMP.h
 */

#ifndef __PREVIEW_ZMP_H__
#define __PREVIEW_ZMP_H__

#include <Eigen/Core>


/**
 * @class previewZMP
 */
class previewZMP {

 public:

  previewZMP();
  ~previewZMP();

  void setDynamics( double _dt,
		    double _z_COM );

  void setLQIGains();
  void calculateControllerGains();
  void printMatrices();

  // Dynamics
  double mdt;
  double mG;

  Eigen::MatrixXd mA;
  Eigen::MatrixXd mB;
  // Output
  Eigen::MatrixXd mC;

  // Controller
  Eigen::MatrixXd mQe;
  Eigen::MatrixXd mQx;
  Eigen::MatrixXd mR;
  
  // Controller helpers
  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_C;

  Eigen::MatrixXd m_Q;
  Eigen::MatrixXd m_P;
  Eigen::MatrixXd m_K;

};

#endif /** __PREVIEW_ZMP_H__ */
