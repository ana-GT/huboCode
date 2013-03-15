/**
 * @file test.cpp
 * @author A. Huaman
 */

#include "basicModel.h"


int main( int argc, char* argv[] ) {

  basicModel bm;

  Eigen::MatrixXd A, B, C;
  std::vector<Eigen::VectorXd> u, x, y;
  Eigen::VectorXd x0;
  int xSize = 3;
  int numSteps = 400;

  double zc = 0.6;
  double g = 9.81;
  double dt = 0.005;

  // Set matrices
  A = Eigen::MatrixXd::Zero( xSize, xSize );
  A(0,1) = 1; A(1,2) = 1;

  B = Eigen::MatrixXd::Zero( xSize, 1 );
  B(2,0) = 1;

  C = Eigen::MatrixXd( 1, xSize );
  C(0,0) = 1; C(0,2) = -zc/g;

  // Set initial state
  x0 = Eigen::VectorXd::Zero(3);
  
  // Run it
  Eigen::VectorXd ut(1);
  ut(0) = 1;
  u = std::vector<Eigen::VectorXd>( numSteps, ut );

  bm.initModel( A, B, C );
  bm.simulateRun( x0, dt, numSteps, u, x, y );

  return 0;
}
