/*
 * LineFollow.cpp
 *
 *  Created on: Feb 22, 2019
 *      Author: jaconkliun
 */

#include "LineFollow.h"
#include "Arduino.h"

LineFollow::~LineFollow() {
  // do nothing
}
	LineFollow::LineFollow(DrivingChassis *achassis){
		this->chassis = achassis;
    this->sensor1Val = 0;
    this->sensor2Val = 0;
    this->lineCount = 0;
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
	void LineFollow::followLine(){
		if(sensor1Val >= BLACK && sensor2Val <= WHITE){
		chassis->turnDegrees(-30, 0);
     //Serial.printf("Sen1:BLACK,Sen2:WHITE\n", sensor1Val, sensor2Val);
     //Serial.println("Turning Right");
		}
		else if(sensor1Val <= WHITE && sensor2Val >= BLACK){
			chassis->turnDegrees(30, 0);
      //Serial.printf("Sen1:WHITE,Sen2:BLACK\n", sensor1Val, sensor2Val);
      //Serial.println("Turning Left");
		}
		else if(sensor1Val >= BLACK && sensor2Val >= BLACK){
      //Serial.println("Horizontal Line");
      //Serial.printf("Sen1:BLACK,Sen2:BLACK\n", sensor1Val, sensor2Val);
					lineCount++;
		}
		else if(sensor1Val <= WHITE && sensor2Val <= WHITE){
      //Serial.printf("Sen1:WHITE,Sen2:WHITE\n", sensor1Val, sensor2Val);
      //Serial.println("Moving Forward");
			chassis->driveForward(10, 0);
     
		}
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
