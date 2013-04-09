/**
 * @file previewZMP.cpp
 * @brief
 */

#include "previewZMP.h"
#include <iostream>

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
  
  // _C: 2x8
  m_C = Eigen::MatrixXd::Zero( 2, 8 );
  m_C.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );

  
  // _Q: 4x4
  m_Q = Eigen::MatrixXd::Zero( 8, 8 );
  m_Q.block( 0, 0, 2, 2 ) = mQe;
  m_Q.block( 2, 2, 6, 6 ) = mQx;
  
  

  // _P : 8x8
  m_P = Eigen::MatrixXd::Zero( 8, 8 ); 
  m_P << 58.848, 0.000, 1702.1,  0.000, 494.79,  0.000,  2.4956, 0.000,
    0.000,   58.848,    0.000,   1702.1,    0.000,   494.79,     0.000,   2.4956,
    1702.1,    0.000,    50689,    0.000,    14740,    0.000,    75.799,   0.000,
    0.000,   1702.1,    0.000,    50689,    0.000,    14740,     0.000,   75.799,
    494.79,    0.000,    14740,    0.000,   4286.3,    0.000,    22.067,   0.000,
    0.000,   494.79,    0.000,    14740,    0.000,   4286.3,    0.000,   22.067,
    2.4956,    0.000,   75.799,    0.000,   22.067,    0.000,    0.12092,   0.000,
    0.000,   2.4956,  0.000,   75.799,    0.000,   22.067,    0.000,   0.12092;

  Eigen::MatrixXd temp = ( mR + m_B.transpose()*m_P*m_B );//.inverse();
  Eigen::MatrixXd tempInv(2,2);
  double a, b, c, d;
  a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
  tempInv << d, -b, -c, a;
  tempInv = tempInv*( 1.0/(a*d - b*c) );

  m_K = tempInv*(m_B.transpose())*m_P*m_A;
}

/**
 * @function previewZMP
 */
void previewZMP::printMatrices() {

  std::cout << "A: \n"<< mA << std::endl;
  std::cout << "B: \n"<< mB << std::endl;
  std::cout << "C: \n"<< mC << std::endl;


  std::cout << "~A: \n"<< m_A << std::endl;
  std::cout << "~B: \n"<< m_B << std::endl;
  std::cout << " R: \n"<< mR << std::endl;
  std::cout << "~Q: \n"<< m_Q << std::endl;

  std::cout << "~P: \n"<< m_P << std::endl;
  std::cout << " temp Inv: " << tempInv << std::endl;
  std::cout << "~K: \n"<< m_K << std::endl;
}
