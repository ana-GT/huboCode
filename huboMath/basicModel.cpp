/**
 * @file basicModel.cpp
 */

#include "basicModel.h"
#include <stdio.h>

/**
 * @function basicModel
 * @brief Constructor
 */
basicModel::basicModel() {

}

/**
 * @function ~basicModel
 * @brief Destructor
 */
basicModel::~basicModel() {

}

/**
 * @function initModel
 * @brief Init system matrices
 */
void basicModel::initModel( Eigen::MatrixXd _A, Eigen::MatrixXd _B,
			    Eigen::MatrixXd _C ) {

  mA = _A;
  mB = _B;
  mC = _C;
}

/**
 * @function simulateRun
 * @brief Simulate and store system state and output given the settings
 */
bool basicModel::simulateRun( Eigen::VectorXd _x0, double _dt,
			      int _numSteps,
			      std::vector<Eigen::VectorXd> _u,
			      std::vector<Eigen::VectorXd> _x,
			      std::vector<Eigen::VectorXd> _y ) {
  // Set things up
  _x.resize(0);
  _y.resize(0);
  mdt = _dt;

  // Init state to _x0
  mX = _x0;

  // Checking time size
  int steps = _numSteps;
 
  if( _u.size() != steps ) {
    printf( "Insufficient input signal info! Exiting \n" );
    return false;
  }

  // Simulate
  Eigen::VectorXd dx;

  for( int i = 0; i < steps; ++i ) {
    
    dx = mA*mX + mB*_u[i];
    mY = mC*mX;

    _x.push_back( mX );
    _y.push_back( mY );    

    // Update
    mX = mX + dx*mdt;
  }
  
  return true;
}
