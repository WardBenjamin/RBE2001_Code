/*
 * LineFollow.cpp
 *
 *  Created on: Feb 22, 2019
 *      Author: jaconkliun
 */

#include "LineFollow.h"
#include "Arduino.h"
#include "StudentsRobot.h"

LineFollow::~LineFollow() {
  // do nothing
}
	LineFollow::LineFollow(DrivingChassis *achassis){
		this->chassis = achassis;
    this->sensor1Val = 0;
    this->sensor2Val = 0;
    this->lineCount = 0;
    this->numLines = 0;
	}
	/**
	 * read read sensor(s) and update values
	 */
	void LineFollow::readSensors(){
		sensor1Val = analogRead(LINE_SENSE_ONE);
		sensor2Val = analogRead(LINE_SENSE_TWO);
    Serial.printf("Sen1:%d,Sen2:%d\n", sensor1Val, sensor2Val);
	}
	/**
	 * followLine take appropriate chassis action based on sensor values
	 */
	void LineFollow::followLineDetect(RobotStateMachine * myStatus, RobotStateMachine * myNextStatus, RobotStateMachine waitStatus, RobotStateMachine driveStatus, RobotStateMachine countStatus, RobotStateMachine doneStatus, int correctionAmt){
		if(sensor1Val >= BLACK && sensor2Val >= BLACK){
		      lineCount++;
		      if(lineCount >= numLines){
		      *myNextStatus = driveStatus;
		      *myStatus = doneStatus;
		      }
		      else{
		    	  *myStatus = countStatus;
		      }

		    }
		    else{
		      if(sensor1Val >= BLACK && sensor2Val <= WHITE){
		    chassis->myleft->startInterpolationDegrees(chassis->myleft->getAngleDegrees() + correctionAmt, 1000, SIN);
		    }
		    else if(sensor1Val <= WHITE && sensor2Val >= BLACK){
		    	chassis->myright->startInterpolationDegrees(chassis->myright->getAngleDegrees() + correctionAmt, 1000, SIN);
		    }
		    *myNextStatus = driveStatus;
		    *myStatus = waitStatus;
		    }
	}


	void LineFollow::followLineDrive(RobotStateMachine * myStatus, RobotStateMachine * myNextStatus, RobotStateMachine detectStatus, RobotStateMachine waitStatus,int forwardAmt){
		chassis->driveForward(forwardAmt, 1000);
		*myNextStatus = detectStatus;
		*myStatus = waitStatus;
	}

	/**
	 * getLineCount return current number of counted perpendicular lines
	 * @return number of perpendicular lines counted since last resetCount()
	 */
	int LineFollow::getLineCount(){
		return lineCount;
	}
	/**
	 * resetLineCount reset the number of counted perpendicular lines
	 */
	void LineFollow::resetLineCount(){
		lineCount = 0;
	}
