/**
 * @file previewZMP.h
 */

#include "previewZMP.h"

int main ( int argc, char* argv[] ) {

  previewZMP pz;
  double zr = 0.8; /**< Z value for CoM */
  double g = 9.81; /**< Gravity */
  double dt = 0.01; /**< Time step */


  // LQI
  pz.setDynamics( dt, zr );
  pz.setLQIGains( );

  pz.calculateControllerGains();
  pz.printMatrices();

  

  return 0;

}

/**
 * @function generateSteps 
 */
void generateSteps( double _totalTime,
		    double _stepTime,
		    double _dt,
		    double _stepDistance,
		    std::vector<double> _px,
		    std::vector<double> _py ) {
  
  int numSteps = _totalTime / _stepTime;

  _px.resize( 0 );
  _py.resize( 0 );

  for( int i = 0; i < numSteps; ++i ) {
    
    _px[0] = _stepDistance*step;
  }

}
