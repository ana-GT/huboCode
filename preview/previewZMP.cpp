/**
 * @file previewZMP.cpp
 * @brief
 */

#include "previewZMP.h"
#include <iostream>
#include <stdio.h>

/**
 * @function previewZMP
 * @brief Constructor
 */
previewZMP::previewZMP() {

  mG = 9.81;
}

/**
 * @function ~previewZMP
 * @brief Destructor
 */
previewZMP::~previewZMP() {

}

/**
 * @function setDynamics
 * @brief Set dynamics and output matrix
 */
void previewZMP::setDynamics( double _dt,
			      double _z_COM ) {

  mdt = _dt;

  mA = Eigen::MatrixXd::Zero( 6, 6 );
  mA.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 2, 2, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 4, 4, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );

  mA.block( 0, 2, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 0, 4, 2, 2 ) = (mdt*mdt/2.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mA.block( 2, 4, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );

  mB = Eigen::MatrixXd::Zero( 6, 2 );
  mB.block( 0, 0, 2, 2 ) = (mdt*mdt*mdt/6.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mB.block( 2, 0, 2, 2 ) = (mdt*mdt/2.0)*Eigen::MatrixXd::Identity( 2, 2 );
  mB.block( 4, 0, 2, 2 ) = mdt*Eigen::MatrixXd::Identity( 2, 2 );

  mC = Eigen::MatrixXd::Zero( 2, 6 );
  mC.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  mC.block( 0, 4, 2, 2 ) = -(_z_COM / mG )*Eigen::MatrixXd::Identity( 2, 2 );

}

/**
 * @function setLQIGains
 * @brief Set gain matrices for the optimal controller
 */
void previewZMP::setLQIGains() {

  mQe = Eigen::MatrixXd::Identity( 2, 2 );
  mQx = Eigen::MatrixXd::Zero( 6, 6 );
  mR = 0.000001*Eigen::MatrixXd::Identity( 2, 2 );
 
}

/**
 * @function calculateControllerGains
 * @brief Calculate..nah, self-explanatory
 */
void previewZMP::calculateControllerGains() {

  // _A: 8x8
  m_A = Eigen::MatrixXd::Zero( 8, 8 );
  m_A.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  m_A.block( 0, 2, 2, 6 ) = mC*mA;
  m_A.block( 2, 2, 6, 6 ) = mA;

  // _B: 8x2
  m_B = Eigen::MatrixXd::Zero( 8, 2 );
  m_B.block( 0, 0, 2, 2 ) = mC*mB;
  m_B.block( 2, 0, 6, 2 ) = mB;
  
  // _Q: 4x4
  m_Q = Eigen::MatrixXd::Zero( 8, 8 );
  m_Q.block( 0, 0, 2, 2 ) = mQe;
  m_Q.block( 2, 2, 6, 6 ) = mQx;

  // _I: 8x2
  m_I = Eigen::MatrixXd::Zero(8,2);
  m_I.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2);
  
  // K : 8x8 from Octave dare(A,B,Q,R) - Discrete Riccati Equation
  mK = Eigen::MatrixXd::Zero( 8, 8 ); 
  mK << 58.848, 0.000, 1702.1,  0.000, 494.79,  0.000,  2.4956, 0.000,
    0.000,   58.848,    0.000,   1702.1,    0.000,   494.79,     0.000,   2.4956,
    1702.1,    0.000,    50689,    0.000,    14740,    0.000,    75.799,   0.000,
    0.000,   1702.1,    0.000,    50689,    0.000,    14740,     0.000,   75.799,
    494.79,    0.000,    14740,    0.000,   4286.3,    0.000,    22.067,   0.000,
    0.000,   494.79,    0.000,    14740,    0.000,   4286.3,    0.000,   22.067,
    2.4956,    0.000,   75.799,    0.000,   22.067,    0.000,    0.12092,   0.000,
    0.000,   2.4956,  0.000,   75.799,    0.000,   22.067,    0.000,   0.12092;

  Eigen::MatrixXd temp = ( mR + m_B.transpose()*mK*m_B );
  Eigen::MatrixXd tempInv(2,2);
  double a, b, c, d;
  a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
  tempInv << d, -b, -c, a;
  tempInv = tempInv*( 1.0/(a*d - b*c) );

  // Verifying Riccati
  Eigen::MatrixXd verif = m_A.transpose()*mK*m_A - m_A.transpose()*mK*m_B*tempInv*m_B.transpose()*mK*m_A + m_Q;
  std::cout <<"Verif K: \n" << verif << std::endl;
  std::cout << " K from octave: \n" <<  mK<< std::endl;


  // W
  mW = tempInv*m_B.transpose();

  // G1
  mG1 = mW*mK*m_I;

  // G2
  mG2 = mW*mK*m_A.block( 0, 2, 8, 6 );
  
  // G3
  mG3.resize(0);
  Eigen::MatrixXd G3 = Eigen::MatrixXd::Zero( 2, 2 );

  Eigen::MatrixXd exp = Eigen::MatrixXd::Identity(8,8);
  Eigen::MatrixXd factor1 = ( m_A - m_B*mW*mK*m_A ).transpose();
  
  // i = 0
  mG3.push_back( Eigen::MatrixXd::Zero(2,2) );
  
  // i = 1
  G3 = -mW*exp*mK*m_I;
  mG3.push_back( G3 );

  // i = 2 till 300 ?
  for( int i = 2; i < 300; ++i ) {
    exp = exp*factor1;
    G3 = -mW*exp*mK*m_I;
    mG3.push_back( G3 );
  }

}

/**
 * @function runController
 */
void previewZMP::runController() {

  // x: [ pos_x, pos_y, vel_x, vel_y, acc_x, acc_y ]^T  
  Eigen::MatrixXd x(6,1);
  // y: [ zmp_x, zmp_y ]^T
  Eigen::MatrixXd y(2,1);
  // u: [ u_x, u_y ]^T
  Eigen::MatrixXd u(2,1);

  Eigen::MatrixXd G3;
  Eigen::MatrixXd old_x;

  // t = 0
  x << 0, 0, 0, 0, 0, 0;
  y = mC*x;
  u << 0, 0;
  old_x = x;
  
  // Add
  mX.push_back( x.block(0,0,2,1) );
  mY.push_back( y );
  mU.push_back( u );

  // t = 1*dt to ... 
  int N = (int) ( mStepTime / mdt );

  for( int i = 0; i < mZMP.size() - 2*N - 1; ++i ) {
    x = mA*x + mB*u;
    y = mC*x;

    // preview factor
    G3 = Eigen::MatrixXd::Zero(2,1);
    for( int j = 1; j <= 2*N; ++j ) {
      G3 = G3 + mG3[j]*( mZMP[i+j] - mZMP[i+j-1] );
    }
    
    u = u - mG1*(y - mZMP[i]) - mG2*(x - old_x) - G3;  

    old_x = x;

    // Add
    mX.push_back( x.block(0,0,2,1) );
    mY.push_back( y );
    mU.push_back( u );
  }
}

/**
 * @function printMatrices
 */
void previewZMP::printMatrices() {

  std::cout << "A: \n"<< mA << std::endl;
  std::cout << "B: \n"<< mB << std::endl;
  std::cout << "C: \n"<< mC << std::endl;


  std::cout << "~A: \n"<< m_A << std::endl;
  std::cout << "~B: \n"<< m_B << std::endl;
  std::cout << " R: \n"<< mR << std::endl;
  std::cout << "~Q: \n"<< m_Q << std::endl;

  std::cout << "K: \n"<< mK << std::endl;
}

/**
 * @function generateSteps 
 */
void previewZMP::generateSteps( double _totalTime,
				double _stepTime,
				double _stepDistance,
				double _footSeparation ) {
  
  
  int numTimeSteps = (int) ( _totalTime / mdt );
  mStepTime = _stepTime;
  mStepDistance = _stepDistance;
  mFootSeparation = _footSeparation;

  double t;
  int n;
  Eigen::MatrixXd zmp(2,1);
  mZMP.resize(0);


  // Generate steps
  for( int k = 0; k < numTimeSteps; ++k ) {

    t = k*mdt;    
    n = floor( t / _stepTime );
    
    zmp(0,0) = n*_stepDistance;
    if( n == 0 ) {
      zmp(1,0) = 0;
    }
    else {
      if( n%2 == 0 ) { zmp(1,0) = -1*_footSeparation/2.0; }
      else { zmp(1,0) = 1*_footSeparation/2.0; }
    }
    mZMP.push_back( zmp );
  }

}

/**
 * @function printPlottingData
 */
void previewZMP::printPlottingData() {

  printf("Print plotting data \n");
  FILE* mP;

  // ZMP
  mP = fopen( "zmp.txt", "w" );
  for( int i = 0; i < mZMP.size(); ++i ) {
    fprintf( mP, "%f %f %f \n", mdt*i, mZMP[i](0,0), mZMP[i](1,0) );
  }
  fclose( mP );

  // Gi
  mP = fopen( "G3.txt", "w" );
  for( int i = 0; i < mG3.size(); ++i ) {
    fprintf( mP, "%f %f %f %f %f\n", mdt*i, 
	     -1*mG3[i](0,0),  -1*mG3[i](1,1),
	     -1*mG3[i](0,1),  -1*mG3[i](1,0) );
  }
  fclose( mP ); 

  // x, y, u
  mP = fopen( "xyu.txt", "w" );
  for( int i = 0; i < mX.size(); ++i ) {
    fprintf( mP, "%f %f %f %f %f %f %f \n", mdt*i, 
	     mX[i](0,0), mX[i](1,0),
	     mY[i](0,0), mY[i](1,0),
	     mU[i](0,0), mU[i](1,0) );
  }
  fclose( mP ); 

}
