/*
 * LineFollow.h
 *
 *  Created on: 2/22/19
 *      Author: jaconklin
 */
#include "config.h"
#include "DriveChassis.h"
#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

class LineFollow {

public:
  virtual ~LineFollow();
	LineFollow(DrivingChassis *achassis);
  int sensor1Val;
  int sensor2Val;
  int lineCount;
  DrivingChassis * chassis;
  static const int BLACK = 2000;
  static const int WHITE = 500;
	/**
	 * read read sensor(s) and update values
	 */
	void readSensors();
	/**
	 * followLine take appropriate chassis action based on sensor values
	 */
	void followLine();
	/**
	 * getLineCount return current number of counted perpendicular lines
	 * @return number of perpendicular lines counted since last resetCount()
	 */
	int getLineCount();
	/**
	 * resetLineCount reset the number of counted perpendicular lines
	 */
	void resetLineCount();
private:
	
	
};

#endif
