/**
 * @file utilities.h
 * @author A. Huaman <ahuaman3@gatech.edu> (adapted from .m files by X. Yan)
 * @date 2013/04/22
 */

#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <iostream>
#include <vector>
#include <string>

/** Generate zmp x and y positions for a straight walk */
void generateZmpPositions( std::vector<double> &_zmpx,
			   std::vector<double> &_zmpy,
			   int _numSteps = 5,
			   const bool &_startLeftFoot = true,
			   const double &_stepLength = 0.1,
			   const double &_footSeparation = 0.282,
			   const int &_slope = 30,
			   const int &_level = 210,
			   const int &_wait = 6000 );

/** Generate a nice step function with spline blendings between transitions */
std::vector<double> generateSmoothPattern( const double &_x0,
					   const double &_xf,
					   const int &_numTransitionPts,
					   const int &_numConstantPts );


/** Print */
void print( const std::string &_name,
	    const std::vector<double> &_zmpx,
	    const std::vector<double> &_zmpy );

#endif /** _UTILITIES_H_ */
