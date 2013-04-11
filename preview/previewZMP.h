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
  void runController();

  void printMatrices();

  void generateSteps( double _totalTime,
		      double _stepTime,
		      double _stepDistance,
		      double _footSeparation );

  void printPlottingData();

  // Dynamics
  double mdt;
  double mG;

  // Step info
  double mStepTime;
  double mStepDistance;
  double mFootSeparation;

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

  Eigen::MatrixXd m_Q;
  Eigen::MatrixXd m_I;
  Eigen::MatrixXd mW;
  Eigen::MatrixXd mK;
  Eigen::MatrixXd mG1;
  Eigen::MatrixXd mG2;


  // Preview gain calculation
  std::vector<Eigen::MatrixXd> mG3;

  // ZMP Steps
  std::vector<Eigen::VectorXd> mZMP;

  // Stored x, y and u
  std::vector<Eigen::VectorXd> mX;
  std::vector<Eigen::VectorXd> mY;
  std::vector<Eigen::VectorXd> mU;

};

#endif /** __PREVIEW_ZMP_H__ */
