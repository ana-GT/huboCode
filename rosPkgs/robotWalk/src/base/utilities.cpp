/** 
 * @file utilities.cpp
 * @brief Functions to calculate preview control stuff and foot print generation
 */
#include "utilities.h"


#include <stdlib.h>
#include <stdio.h>

/**
 * @function generateZmpPositions
 */
void generateZmpPositions( std::vector<double> &_zmpx,
			   std::vector<double> &_zmpy,
			   int _numSteps,
			   const bool &_startLeftFoot,
			   const double &_stepLength,
			   const double &_footSeparation,
			   const int &_slope,
			   const int &_level,
			   const int &_wait ) {

  std::vector<double> temp;
  double start; double end;

  //-- Prepare the vectors
  _zmpx.resize(0); _zmpy.resize(0);

  //-- Generate ZMP X component
  for( unsigned int i = 0; i < _numSteps; ++i ) {

    // Start at the center of both feet (double support)
    if( i == 0 ) { start = i*_stepLength; end = i*_stepLength; } 
    // Ends at the center of both feet (double support)
    else if( i == _numSteps - 1 ) { start = (i-1)*_stepLength; end = (i-1)*_stepLength; }
    // Normal walk is single support so keep changing feet
    else { start = (i-1)*_stepLength; end = i*_stepLength;  }    

    temp = generateSmoothPattern( start, end, _slope, _level );
    _zmpx.insert( _zmpx.end(), temp.begin(), temp.end() );

  } // for

  //-- Generate ZMP Y component
  double leftFoot = _footSeparation / 2.0;
  double rightFoot = -1*_footSeparation / 2.0;
  
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

    temp = generateSmoothPattern( start, end, _slope, _level );
    _zmpy.insert( _zmpy.end(), temp.begin(), temp.end() );

    tempFoot = currentFoot;
    currentFoot = nextFoot;
    nextFoot = tempFoot;
  } // end for
  
  
}



/**
 * @function generateSmoothPattern
 */
std::vector<double> generateSmoothPattern( const double &_x0,
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
 * @function print
 */
void print( const std::string &_name,
	    const std::vector<double> &_zmpx,
	    const std::vector<double> &_zmpy ) {

  FILE* pFile;

  pFile = fopen( _name.c_str(), "w" );

  for( int i = 0; i < _zmpx.size(); ++i ) {
    fprintf( pFile, "%d %f %f \n", i, _zmpx[i], _zmpy[i] );
  }

  fclose( pFile );

}
