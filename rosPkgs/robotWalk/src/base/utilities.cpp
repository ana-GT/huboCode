/** 
 * @file utilities.cpp
 * @brief Functions to calculate preview control stuff and foot print generation
 */
#include "utilities.h"

#include <stdlib.h>
#include <stdio.h>

/**
 * @function zmpUtilities
 * @brief Constructor
 */
zmpUtilities::zmpUtilities() {

}

/**
 * @function ~zmpUtilities
 * @brief Destructor
 */
zmpUtilities::~zmpUtilities() {

}

/**
 * @function setParameters
 */
void zmpUtilities::setParameters( const double &_dt,
				  const double &_g ) {
  mdt = _dt;
  mG = _g;
}

/**
 * @function generateZmpPositions
 */
void zmpUtilities::generateZmpPositions( int _numSteps,
					 const bool &_startLeftFoot,
					 const double &_stepLength,
					 const double &_footSeparation,
					 const double &_stepDuration,
					 const double &_slopeTime,
					 const double &_levelTime,
					 const int &_numWaitSteps ) {
  std::vector<double> temp;
  double start; double end;
  int numSlopePts, numLevelPts;

  //-- Prepare the vectors
  std::vector<double> zmpx, zmpy;

  //-- Set a few variables
  mStepLength = _stepLength;
  mFootSeparation = _footSeparation;
  mStepDuration = _stepDuration;
  
  numSlopePts = _slopeTime / mdt;
  numLevelPts = _levelTime / mdt;
  
  //-- Generate ZMP X component
  for( unsigned int i = 0; i < _numSteps; ++i ) {

    // Start at the center of both feet (double support)
    if( i == 0 ) { start = i*mStepLength; end = i*mStepLength; } 
    // Ends at the center of both feet (double support)
    else if( i == _numSteps - 1 ) { start = (i-1)*mStepLength; end = (i-1)*mStepLength; }
    // Normal walk is single support so keep changing feet
    else { start = (i-1)*mStepLength; end = i*mStepLength;  }    

    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpx.insert( zmpx.end(), temp.begin(), temp.end() );

  } // for

  //-- Generate ZMP Y component
  double leftFoot = mFootSeparation / 2.0;
  double rightFoot = -1*mFootSeparation / 2.0;
  
  double currentFoot; double nextFoot; double tempFoot;
  
  if( _startLeftFoot == true ) {
    currentFoot = rightFoot;
    nextFoot = leftFoot;
  }

  for( unsigned int i = 0; i < _numSteps; ++i ) {

    // Start at the center of both feet (double support)
    if( i == 0 ) { start = 0; end = nextFoot; } 
    // Ends at the center of both feet (double support)
    else if( i == _numSteps - 1 ) { start = currentFoot; end = 0; }
    // Normal walk is single support so keep changing feet
    else { start = currentFoot; end = nextFoot;   }    

    temp = generateSmoothPattern( start, end, numSlopePts, numLevelPts );
    zmpy.insert( zmpy.end(), temp.begin(), temp.end() );

    tempFoot = currentFoot;
    currentFoot = nextFoot;
    nextFoot = tempFoot;
  } // end for
    
  // Store
  mZMP.resize(0);
  Eigen::MatrixXd zmp(2,1);
  for( int i = 0; i < zmpy.size(); ++i ) {
    zmp(0,0) = zmpx[i];
    zmp(1,0) = zmpy[i];
    mZMP.push_back( zmp );
  }

}


/**
 * @function generateSmoothPattern
 */
std::vector<double> zmpUtilities::generateSmoothPattern( const double &_x0,
							 const double &_xf,
							 const int &_numTransitionPts,
							 const int &_numConstantPts ) {
  
  int numTotalPts = _numTransitionPts + _numConstantPts;
  std::vector<double> pts;

  // Set all the points to constant value (the transition pts come below)
  pts.assign( numTotalPts, _xf );


  // Generate transition points
  double t; double p;
  for( int i = 0; i < _numTransitionPts; ++i ) {
    t = (double) i / ( (double) (_numTransitionPts - 1) );
    p = ( 1 - t )*_x0 + t*_xf + t*(1-t)*( (_x0-_xf)*(1-t) + (_xf-_x0)*t ); 
    pts[i] = p;
  }

  return pts;
}


/**
 * @function getControllerGains
 */
void zmpUtilities::getControllerGains( const double &_Qe,
				       const double &_R,
				       const double &_z_COM,
				       const int &_numPreviewSteps ) {

  // Controller
  Eigen::MatrixXd Qe, Qx, R;
  
  // Controller helpers
  Eigen::MatrixXd _A, _B;
  Eigen::MatrixXd _Q, _I, W, K;

  //-- Store Num preview steps
  mN = _numPreviewSteps*( mStepDuration / mdt );
  mzCOM = _z_COM;

  //-- Set dynamics
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
  mC.block( 0, 4, 2, 2 ) = -(mzCOM / mG )*Eigen::MatrixXd::Identity( 2, 2 );

  
  //-- Set LQI gains for optimal controller
  Qe = _Qe*Eigen::MatrixXd::Identity( 2, 2 );
  Qx = Eigen::MatrixXd::Zero( 6, 6 );
  R = _R*Eigen::MatrixXd::Identity( 2, 2 );
  

  //-- Set auxiliar matrices to calculate preview controller gains
  // _A: 8x8
  _A = Eigen::MatrixXd::Zero( 8, 8 );
  _A.block( 0, 0, 2, 2 ) = Eigen::MatrixXd::Identity( 2, 2 );
  _A.block( 0, 2, 2, 6 ) = mC*mA;
  _A.block( 2, 2, 6, 6 ) = mA;

  // _B: 8x2
  _B = Eigen::MatrixXd::Zero( 8, 2 );
  _B.block( 0, 0, 2, 2 ) = mC*mB;
  _B.block( 2, 0, 6, 2 ) = mB;
  
  // _Q: 8x8
  _Q = Eigen::MatrixXd::Zero( 8, 8 );
  _Q.block( 0, 0, 2, 2 ) = Qe;
  _Q.block( 2, 2, 6, 6 ) = Qx;

  // _I: 8x2
  _I = Eigen::MatrixXd::Zero(8,2);
  _I.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2);
  
  iterative_DARE( K, _A, _B, _Q, R );

  Eigen::MatrixXd temp = ( R + _B.transpose()*K*_B );
  Eigen::MatrixXd tempInv(2,2);
  double a, b, c, d;
  a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
  tempInv << d, -b, -c, a;
  tempInv = tempInv*( 1.0/(a*d - b*c) );

  std::cout << "K from Iterative: \n" << K <<std::endl;

  // W
  W = tempInv*_B.transpose();

  // G1
  mG1 = W*K*_I;

  // G2
  mG2 = W*K*_A.block( 0, 2, 8, 6 );
  
  // G3
  mG3.resize(0);
  Eigen::MatrixXd G3i = Eigen::MatrixXd::Zero( 2, 2 );

  Eigen::MatrixXd exp = Eigen::MatrixXd::Identity(8,8);
  Eigen::MatrixXd factor1 = ( _A - _B*W*K*_A ).transpose();
  
  // i = 0
  mG3.push_back( G3i );
  
  // i = 1
  G3i = -W*exp*K*_I;
  mG3.push_back( G3i );

  // i = 2 till _numPreviewSteps]
  for( int i = 2; i <= mN; ++i ) {
    exp = exp*factor1;
    G3i = -W*exp*K*_I;
    mG3.push_back( G3i );
  }
  
}

/**
 * @function iterative_DARE
 */
bool zmpUtilities::iterative_DARE( Eigen::MatrixXd &_P, 
				   const Eigen::MatrixXd &_A,
				   const Eigen::MatrixXd &_B,
				   const Eigen::MatrixXd &_Q,
				   const Eigen::MatrixXd &_R,
				   const double &_error_threshold,
				   const int &_numIter ) {
  
  Eigen::MatrixXd At = _A.transpose();
  Eigen::MatrixXd Bt = _B.transpose();
  Eigen::MatrixXd temp; 
  Eigen::MatrixXd tempInv( 2, 2 );
  double a, b, c, d;

  Eigen::MatrixXd Pnew, Pt;

  _P = Eigen::MatrixXd::Identity(8,8);
  Pnew = _P; Pt = _P.transpose();
  
  double error;

  for( int i = 0; i < _numIter; ++i ) {
    temp = ( _R + Bt*_P*_B );
    a = temp(0,0); b = temp(0,1); c = temp(1,0); d = temp(1,1);
    tempInv << d, -b, -c, a;
    tempInv = tempInv*( 1.0/(a*d - b*c) );


    Pnew = At*_P*_A - At*_P*_B*tempInv*Bt*Pt*_A + _Q;
    error = ( (Pnew - _P).norm() ) / ( Pnew.norm() );
    _P = Pnew;
    Pt = _P.transpose();
    if( error < _error_threshold ) {
      printf("DARE converges after %d iterations with an error of %f \n", i, error );
      return true;
    }

  }

  printf("DARE did NOT converge! threshold error: %f Last error in iter[%d]: %f \n", _error_threshold,
	 _numIter, error );
  return false;
  
}


/**
 * @function generateCOMPositions
 */
void zmpUtilities::generateCOMPositions( ) {

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
  mX.resize(0); mY.resize(0); mU.resize(0);
  mX.push_back( x.block(0,0,2,1) );
  mY.push_back( y );
  mU.push_back( u );
 
  // t = 1*dt to ... 
  for( int i = 0; i < mZMP.size() - mN - 1; ++i ) {

    x = mA*x + mB*u;
    y = mC*x;

    // preview factor
    G3 = Eigen::MatrixXd::Zero(2,1);

    for( int j = 1; j < mN; ++j ) {
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
 * @function print
 */
void zmpUtilities::print( const std::string &_name,
			  const std::vector<Eigen::VectorXd> &_zmp ) {

  FILE* pFile;

  pFile = fopen( _name.c_str(), "w" );

  for( int i = 0; i < _zmp.size(); ++i ) {
    fprintf( pFile, "%d %f %f \n", i, _zmp[i](0,0), _zmp[i](1,0) );
  }

  fclose( pFile );

}

