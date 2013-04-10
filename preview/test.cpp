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

  pz.generateSteps( 6.0, 1.0, 0.15, 0.3 );
  pz.printPlottingData();

  return 0;

}

