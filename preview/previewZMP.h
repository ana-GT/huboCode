/**
 * @file previewZMP.h
 */

#ifndef __PREVIEW_ZMP_H__
#define __PREVIEW_ZMP_H__

#include <Eigen/Core>
#include <vector>

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

  void generateSteps( double _totalTime,
		      double _stepTime,
		      double _stepDistance,
		      double _footSeparation );

  void printPlottingData();

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
  Eigen::MatrixXd m_Ac;

  // Preview gain calculation
  std::vector<Eigen::MatrixXd> mGi;
  Eigen::MatrixXd mKe;
  Eigen::MatrixXd mKx;

  // ZMP Steps
  std::vector<double> mZMPx;
  std::vector<double> mZMPy;

};

#endif /** __PREVIEW_ZMP_H__ */
