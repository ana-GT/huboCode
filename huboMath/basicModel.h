/**
 * @file basicModel.h
 * @brief Simple inverted pendulum model for walking
 * @author A. Huaman
 * @date 2013-03-14
 */
#ifndef __BASIC_MODEL_H__
#define __BASIC_MODEL_H__

#include <Eigen/Core>
#include <vector>
#include <math.h>

/**
 * @class basicModel
 */
class basicModel {

 public:
  basicModel();
  ~basicModel();
  void initModel( Eigen::MatrixXd _A, Eigen::MatrixXd _B,
		  Eigen::MatrixXd _C );
  bool simulateRun( Eigen::VectorXd _x0, double _dt,
		    int _numSteps,
		    std::vector<Eigen::VectorXd> _u,
		    std::vector<Eigen::VectorXd> _x,
		    std::vector<Eigen::VectorXd> _y );

 private: 

  Eigen::MatrixXd mA; /**< Dynamics */
  Eigen::MatrixXd mB; /**<  Input */
  Eigen::MatrixXd mC; /**< Map to output */

  Eigen::VectorXd mU; /**< Input control */
  Eigen::MatrixXd mX; /**< State vector */
  Eigen::VectorXd mY;
  double mdt; /**< Time step */


};

#endif /** __BASIC_MODEL_H__ */
