/**
 * @file pid_controller.cpp
 */

#include <math.h>
#include <stdio.h>
#include <gazebo/math/Helpers.hh>
#include "pid_controller.h"


/**
 * @function pid_controller
 * @brief
 */
pid::controller::pid_controller( double _Kp = 0.0, double _Ki = 0.0, double _Kd = 0.0,
				 double _Imax = 0.0, double _Imin = 0.0,
				 double _outputMax = 0.0, double _outputMin = 0.0 ) 
  : Kp(_Kp), Ki(_Ki), Kd(_Kd), Imax(_Imax), Imin(_Imin), outputMax(_outputMax), outputMin(_outputMin) {
  this->Reset();
}

/**
 * @function pid_controller
 * @brief
 */
pid::controller::~pid_controller() {

}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::init( double _Kp = 0.0, double _Ki = 0.0, double _Kd = 0.0,
			    double _Imax = 0.0, double _Imin = 0.0,
			    double _outputMax = 0.0, double _outputMin = 0.0 ) {

  this->Kp = _Kp;
  this->Ki = _Ki;
  this->Kd = _Kd;
  this->Imax = _Imax;
  this->Imin = _Imin;
  this->outputMax = _outputMax;
  this->outputMin = _outputMin;

  this->Reset();
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setPGain( double _Kp ) {
  Kp = _Kp;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setIGain( double _Ki ) {
  Ki = _Ki;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setDGain( double _Kd ) {
  Kd = _Kd;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setIMax( double _Imax ) {
  Imax = _Imax;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setIMin( double _Imin ) {
  Imin = _Imin;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setOutputMax( double _outputMax ) {
  outputMax = _outputMax;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setOutputMin( double _outputMin ) {
  outputMin = _outputMin;
}

/**
 * @function pid_controller
 * @brief
 */
double pid::controller::update( double _Ep, common::Time _dt ) {
 
  double Xp; double Xd; double Xi;

  this->Ep = _Ep;
  
  if( _dt == common::Time(0,0) || math::isnan(_error) || isinf(_error) ) {
    return 0.0;
  }

  // Calculate P value added to the control signal
  Xp = this->Kp*this->Ep;

  // Calculate I error
  this->Ei = this->Ei + _dt.Double()*this->Ep;

  // Calculate I value added to the control signal
  Xi = this->Ki*this->Ei;

  // Limit Xi so that the limit is meaningful in the control signal
  if( Xi > this->iMax ) {
    Xi = this->iMax;
    this->Ei = Xi / this->Ki;
  }
  else if( Xi < this->iMin ) {
    Xi = this->iMin;
    this->Ei = Xi / this->Ki;
  }

  // Calculate D error
  if( _dt != common::Time(0,0) ) {
    this->Ed = ( this->Ep - this->Ep_prev ) / _dt.Double();
    this->Ep_prev = this->Ep;
  }

  // Calculate D value added to the control signal
  Xd = this->Kd*this->Ed;

  // Add them all together
  this->output = -Xp - Xi - Xd;

  // Check command limits
  if( !math::equal( this->outputMax, 0.0 ) && this->output > this->outputMax ) {
    this->output = this->outputMax;
  }
  if( !math::equal( this->outputMin, 0.0 ) && this->output > this->outputMin ) {
    this->output = this->outputMin;
  }
 
  return this->output;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::setOutput( double _output ) {
  this->output = _output;
}

/**
 * @function pid_controller
 * @brief
 */
double pid::controller::getOutput() {
  return this->output;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::getErrors( double &_Ep, double &_Ei, double &_Ed ) {
  _Ep = this->Ep;
  _Ei = this->Ei;
  _Ed = this->Ed;
}

/**
 * @function pid_controller
 * @brief
 */
void pid::controller::reset() {

  this->Ep_prev = 0.0;
  this->Ep = 0.0;
  this->Ei = 0.0;
  this->Ed = 0.0;
  this->output = 0.0;
  this->outputMax = 0.0;
  this->outputMin = 0.0;
}

