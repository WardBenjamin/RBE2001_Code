/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "Arduino.h"
#include "RBEPID.h"
#include <math.h>

//Class constructor
RBEPID::RBEPID() {

}

//Function to set PID gain values
void RBEPID::setpid(float P, float I, float D) {
	kp = P;
	ki = I;
	kd = D;
}

/**
 * calc the PID control signel
 *
 * @param setPoint is the setpoint of the PID system
 * @param curPosition the current position of the plan
 * @return a value from -1.0 to 1.0 representing the PID control signel
 */
float RBEPID::calc(double setPoint, double curPosition) {
	// calculate error
	float err = setPoint - curPosition;
	// calculate derivative of error
	float derErr = err - last_error;
	// calculate integral error. Running average is best but hard to implement

  //check for sign change and reset integral buffer
  if((last_error != 0) && (err / last_error) < 0){
    clearIntegralBuffer();
  }
  
	sum_error += err;
	// sum up the error value to send to the motor based off gain values.
	float out = (kp * err) + (ki * sum_error) + (kd * derErr);
  last_error = err;

  out = fmin(out, 1);
  out = fmax(out, -1);
	return out;
}

/**
 * Clear the internal representation fo the integral term.
 *
 */
void RBEPID::clearIntegralBuffer() {
	sum_error = 0;
}
