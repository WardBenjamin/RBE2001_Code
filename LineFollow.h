/*
 * LineFollow.h
 *
 *  Created on: 2/22/19
 *      Author: jaconklin
 */
#include "config.h"
#include "DriveChassis.h"
#include "StudentsRobot.h"
#ifndef LINEFOLLOW_H_
#define LINEFOLLOW_H_

class LineFollow {

public:
  virtual ~LineFollow();
	LineFollow(DrivingChassis *achassis);
  int sensor1Val;
  int sensor2Val;
  int lineCount;
  int numLines;
  DrivingChassis * chassis;
  static const int BLACK = 2000;
  static const int WHITE = 1500;
	/**
	 * read read sensor(s) and update values
	 */
	void readSensors();
	/**
	 * followLineDetect take appropriate chassis correction action based on sensor values
	 * @param myStatus current state of the state machine
	 * @param myNextStatus next state of state machine
	 * @param waitStatus status for motors to finish
	 * @param driveStatus state of driving that pairs with detecting
	 * @param countStatus state to go to when hoz. line detected
	 * @param doneStatus state to go to when last hoz. line detected
	 * @param corectionAmt amount to correct wheel pos by when line follow adjust required
	 */
	void followLineDetect(RobotStateMachine * myStatus, RobotStateMachine * myNextStatus, RobotStateMachine waitStatus,
			RobotStateMachine driveStatus, RobotStateMachine countStatus,
			RobotStateMachine doneStatus, int correctionAmt);
	/**
		 * followLineDrive drives forward and bounces to detect
		 * @param myStatus current state of the state machine
		 * @param myNextStatus next state of state machine
		 * * @param driveStatus state of detecting that pairs with driving
		 * @param waitStatus status for motors to finish
		 * @param forwardAmt amountToMove chassis
		 */
	void followLineDrive(RobotStateMachine * myStatus, RobotStateMachine * myNextStatus,RobotStateMachine detectStatus, RobotStateMachine waitStatus,int forwardAmt);
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
