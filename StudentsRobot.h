/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>

#include "DriveChassis.h"

/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {
	StartupRobot = 0, StartRunning = 1, Running = 2, Halting = 3, Halt = 4,WAIT_FOR_MOTORS_TO_FINNISH=5,WAIT_FOR_TIME=6

};
/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};
/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	ServoEncoderPIDMotor * motor1;
	ServoEncoderPIDMotor * motor2;
	HBridgeEncoderPIDMotor * motor3;
	Servo * servo;
	float lsensorVal=0;
	float rsensorVal=0;
	long nextTime =0;
  long startTime =0;
	RobotStateMachine nextStatus = StartupRobot;
public:
	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(ServoEncoderPIDMotor * motor1,
			ServoEncoderPIDMotor * motor2, HBridgeEncoderPIDMotor * motor3,
			Servo * servo);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Ready_for_new_task;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;
	/**
	 * Approve
	 *
	 * @param buffer A buffer of floats containing nothing
	 *
	 * the is the event of the Approve button pressed in the GUI
	 *
	 * This function is called via coms.server() in:
	 * @see RobotControlCenter::fastLoop
	 */
	void Approve(float * buffer);
	/**
	 * ClearFaults
	 *
	 * @param buffer A buffer of floats containing nothing
	 *
	 * this represents the event of the clear faults button press in the gui
	 *
	 * This function is called via coms.server() in:
	 * @see RobotControlCenter::fastLoop
	 */
	void ClearFaults(float * buffer);
	/**
	 * EStop
	 *
	 * @param buffer A buffer of floats containing nothing
	 *
	 * this represents the event of the EStop button press in the gui
	 *
	 * This is called whrn the estop in the GUI is pressed
	 * All motors shuld hault and lock in position
	 * Motors should not go idle and drop the plate
	 *
	 * This function is called via coms.server() in:
	 * @see RobotControlCenter::fastLoop
	 */
	void EStop(float * buffer);
	/**
	 * PickOrder
	 *
	 * @param buffer A buffer of floats containing the pick order data
	 *
	 * buffer[0]  is the material, aluminum or plastic.
	 *
	 * buffer[1]  is the drop off angle 25 or 45 degrees
	 *
	 * buffer[2]  is the drop off position 1, or 2
	 *
	 * This function is called via coms.server() in:
	 * @see RobotControlCenter::fastLoop
	 */
	void PickOrder(float * buffer);

	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();
};

#endif /* STUDENTSROBOT_H_ */
