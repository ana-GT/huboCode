/**
* @file swing.h
* @author A. Huaman and Xinyan Yan
* @date 2013-03-12
*/
#include <stdio.h>

// Hard-coded in a galaxy far, far away...
int const N = 600;

/**< Utility function */
void getSpline3( double _tf, double _x0, double _xf, double _dx0, double _dxf,
double &_a0, double &_a1, double &_a2, double &_a3 );


/**< Simplest case: Cycloid between start and goal, yaw changes linearly*/
void swingSimpleCycloid( double _x0, double _y0, double _z0,
double _x1, double _y1, double _z1,
int _count,
bool _isLeft,
double _pos[N][3],
double _yaw[N] );

/** < Ellipse shape */
void swingEllipse( double _x0, double _y0, double _z0,
double _x1, double _y1, double _z1,
int _count,
bool _isLeft,
double _pos[N][3],
double _yaw[N] );

/**< 2 Cycloids, one for x,y, the other for z. Yaw changes as spline */
void swing2Cycloids( double _x0, double _y0, double _theta0,
	    double _x1, double _y1, double _theta1,
	    int _count, 
	    bool _isLeft,
	    double _pos[N][3],
	    double _yaw[N] );

/**< Simple Bezier in plane xy (2 additional points in yaw directions to guide the curvature) and cycloid in z as usual */
void swingSimpleBezier( double _x0, double _y0, double _theta0,
	    double _x1, double _y1, double _theta1,
	    int _count, 
	    bool _isLeft,
	    double _pos[N][3],
	    double _yaw[N] );

// COMMENTED OUT (EARLIER ATTEMPTS THAT FAILED MISERABLY)

/** < Add a vertical step at start and end. Add a cycloid on top; yaw varies linearly - No continuous :( */
/*
void swingVerticalStep( double _x0, double _y0, double _z0,
double _x1, double _y1, double _z1,
int _count,
bool _isLeft,
double _pos[N][3],
double _yaw[N] );
*/

/** < Add a vertical step at start and end. Add a cycloid on top  and yaw varies as a cubic spline - No continuous :( */
/*
void swingVerticalStepSplineYaw( double _x0, double _y0, double _z0,
double _x1, double _y1, double _z1,
int _count,
bool _isLeft,
double _pos[N][3],
double _yaw[N] );
*/

