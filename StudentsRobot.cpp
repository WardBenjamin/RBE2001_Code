

/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "StudentsRobot.h"



StudentsRobot::StudentsRobot(ServoEncoderPIDMotor * motor1,
		ServoEncoderPIDMotor * motor2, HBridgeEncoderPIDMotor * motor3,
		Servo * servo) {
	Serial.println("StudentsRobot::StudentsRobot called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;

	// Set the PID Clock gating rate. Thie must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 30; // 330hz servo, 3ms update, 30 ms PID
	motor2->myPID.sampleRateMs = 30; // 330hz servo, 3ms update, 30 ms PID
	motor3->myPID.sampleRateMs = 1;  // 10khz H-Bridge, 0.1ms update, 1 ms PID
	// Set default P.I.D gains
	motor1->SetTunings(0.15, 0.0001, 1.6);
	motor2->SetTunings(0.15, 0.0001, 1.6);
	motor3->SetTunings(0.01, 0.0001, 0.6);

	// After attach, compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(0, //the minimum value that the output takes (Full reverse)
			180, //the maximum value the output takes (Full forward)
			90, //the value of the output to stop moving
			1, //a positive value added to the stop value to creep forwards
			1, //a positive value subtracted from stop value to creep backward
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					motor1->encoder.countsMode, // Number of edges that are used to increment the value
			117.5 * // Measured max RPM
					(1 / 60.0) * // Convert to seconds
					(1 / motorToWheel) *  // motor to wheel ratio
					360.0); // convert to degrees
	motor2->setOutputBoundingValues(0, //the minimum value that the output takes (Full reverse)
			180, //the maximum value the output takes (Full forward)
			90, //the value of the output to stop moving
			1, //a positive value added to the stop value to creep forwards
			1, //a positive value subtracted from stop value to creep backward
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					motor2->encoder.countsMode, // Number of edges that are used to increment the value
			117.5 * // Measured max RPM
					(1 / 60.0) * // Convert to seconds
					(1 / motorToWheel) *  // motor to wheel ratio
					360.0); // convert to degrees
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			1, //a positive value added to the stop value to creep forwards
			1, //a positive value subtracted from stop value to creep backward
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					1.0 * // motor to arm stage ratio
					(1.0 / 360.0) * // degrees per revolution
					motor3->encoder.countsMode, // Number of edges that are used to increment the value
			117.5 * // Measured max RPM
					(1 / 60.0) * // Convert to seconds
					360.0); // convert to degrees

chassis = new DrivingChassis(motor1, motor2, 220, 27);
lineFollower = new LineFollow(chassis);

	// Set up the line tracker

	pinMode(LINE_SENSE_ONE, ANALOG);
	pinMode(LINE_SENSE_TWO, ANALOG);
	pinMode(EMITTER_PIN, OUTPUT);
 //pinMode(SERVO_PIN, OUTPUT);
 pinMode(MOTOR3_ENABLE_PIN, OUTPUT);
 pinMode(MOTOR3_DIR, OUTPUT);
servo->setPeriodHertz(50);
servo->attach(SERVO_PIN);
 
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */



 int timeIdx = 0;
 long time1 = 0;
 double values[400][2];
 
void StudentsRobot::updateStateMachine() {
	
	long now = millis();
  
  
	switch (status) {
	case StartupRobot:
    lastStatus = StartupRobot;
    digitalWrite(EMITTER_PIN, HIGH);
    digitalWrite(MOTOR3_ENABLE_PIN, HIGH);
    
    Serial.println("State Machine Startup");
  
  //motor3->startInterpolationDegrees(ENCODER_POS_1, 5000, SIN);
    nextStatus = PickupPanel; //LF_Backup_Init;
   status = WAIT_FOR_APPROVE; //WAIT_FOR_MOTORS_TO_FINISH
		break;
	case StartRunning:
  servo->write(110);
  
  nextTime = millis() + 1000;
  
		lastStatus = StartRunning;
    nextStatus = PickupPanel;
    status = WAIT_FOR_TIME;
		break;
  case PickupPanel:
  
  servo->write(170);
  
  //status = LF_Backup_Init;
  lastStatus = PickupPanel;
  nextTime = millis() + 1000;
  nextStatus = StartRunning;
  status = WAIT_FOR_TIME; 
  break;
	case LF_Backup_Init:
    lastStatus = LF_Backup_Init;
    Serial.println("State: Backup_Init");
    chassis->driveForward(-10, 0);
    status = LF_Backup_Detect;
    
		break;
  case LF_Backup_Detect:
  lastStatus = LF_Backup_Detect;
    Serial.println("State: Backup_Detect");
    lineFollower->readSensors();
    if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val >= lineFollower->BLACK){
      lineCount++;
      if((roofPos == 1 && lineCount == 1) || (roofPos == 2 && lineCount == 2)){
       nextStatus = LF_Transition_1;
      status = STOP;  
      }
      
    }
    else{
      if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val <= lineFollower->WHITE){
    //chassis->turnDegrees(30, 0);
    motor1->startInterpolationDegrees(motor1->getAngleDegrees() - 60, 1000, SIN);
    }
    else if(lineFollower->sensor1Val <= lineFollower->WHITE && lineFollower->sensor2Val >= lineFollower->BLACK){
      //chassis->turnDegrees(-30, 0);
      motor2->startInterpolationDegrees(motor2->getAngleDegrees() - 60, 1000, SIN);
    }
    nextStatus = LF_Backup_Init;
    status = WAIT_FOR_MOTORS_TO_FINISH;  
    }
    break;
  case LF_Transition_1:
  
    Serial.println("State: LF_Transition_1");
    chassis->driveForward(15, 1000);
    nextStatus = LF_Transition_2;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition_1;
    break;
  case LF_Transition_2:
  
    Serial.println("State: LF_Transition_2");
    chassis->turnDegrees(-80, 4000);
    nextStatus = LF_Forward_Init;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition_2;
    break;
  case LF_Forward_Init:
  
    Serial.println("State: LF_Forward_Init");
    chassis->driveForward(50, 1000);
    nextStatus = LF_Forward_Detect;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Forward_Init;
    break;
  case LF_Forward_Detect:
    lastStatus = LF_Forward_Detect;
    nextStatus = LF_Forward_Init;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    Serial.println("State: LF_Forward_Detect");
    lineFollower->readSensors();
    if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val >= lineFollower->BLACK){ //Horizontal Line Detected
      //nextStatus = LF_ForwardInit;
      lineCount++;
      if(lineCount == 2){
        status=Halting;
      }else{
        status = CountLine;  
      }
      
    }
    else{
      if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val <= lineFollower->WHITE){
    chassis->turnDegrees(30, 100);
    //motor1->startInterpolationDegrees(motor1->getAngleDegrees() - 360, 1000, SIN);
    }
    else if(lineFollower->sensor1Val <= lineFollower->WHITE && lineFollower->sensor2Val >= lineFollower->BLACK){
      chassis->turnDegrees(-30, 100);
      //motor2->startInterpolationDegrees(motor2->getAngleDegrees() - 360, 1000, SIN);
    }  
    }
    break;
   case CountLine:
    lastStatus = CountLine;
    chassis->driveForward(15, 1000);
    //nextStatus = LF_Forward_Detect;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = CountLine;
   break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
   lastStatus = WAIT_FOR_TIME;
		break;
	case WAIT_FOR_MOTORS_TO_FINISH:
  //Serial.println("State: WAIT_FOR_MOTORS_TO_FINISH");
  //Serial.println(motor3->getAngleDegrees());
		if (chassis->isChassisDoneDriving() && motor3->isInterpolationDone()) {
			status = nextStatus;
		}
   lastStatus = WAIT_FOR_MOTORS_TO_FINISH;
		break;
  case WAIT_FOR_APPROVE:
  //Serial.println("State: WAIT_FOR_MOTORS_TO_FINISH");
  //Serial.println(motor3->getAngleDegrees());
    if (approve) {
      approve = false;
      status = nextStatus;
    }
    lastStatus = WAIT_FOR_APPROVE;
    break;
  case STOP:
    Serial.println("State: STOP");
    motor2->stop();
    motor1->stop();
    status = nextStatus;
    lastStatus = STOP;
    break;
	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(EMITTER_PIN, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();
		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;
	}
}

/**
 * This is run fast and should return fast
 *
 * You call the PIDMotor's loop function. This will update the whole motor control system
 * This will read from the concoder and write to the motors and handle the hardware interface.
 * Instead of allowing this to be called by the controller yopu may call these from a timer interrupt.
 */
void StudentsRobot::pidLoop() {
	motor1->loop();
	motor2->loop();
	motor3->loop();
}
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
void StudentsRobot::Approve(float * buffer) {
	// approve the procession to new state
	Serial.println("StudentsRobot::Approve");
  approve = true;
	if (myCommandsStatus == Waiting_for_approval_to_pickup) {
		myCommandsStatus = Waiting_for_approval_to_dropoff;
	} else {
		myCommandsStatus = Ready_for_new_task;
	}
}
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
void StudentsRobot::ClearFaults(float * buffer) {
	
  
	Serial.println("StudentsRobot::ClearFaults");
 Serial.printf("status: %d\n",  status);
	myCommandsStatus = Ready_for_new_task;
	status = lastStatus;
}

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
void StudentsRobot::EStop(float * buffer) {
	// Stop the robot immediatly
	Serial.println("StudentsRobot::EStop");
 Serial.printf("status: %d\n",  status);
	myCommandsStatus = Fault_E_Stop_pressed;
	status = Halting;

}
/**
 * PickOrder
 *
 * @param buffer A buffer of floats containing the pick order data
 *
 * buffer[0]  is the material, aluminum or plastic.
 * buffer[1]  is the drop off angle 25 or 45 degrees
 * buffer[2]  is the drop off position 1, or 2
 *
 * This function is called via coms.server() in:
 * @see RobotControlCenter::fastLoop
 */
void StudentsRobot::PickOrder(float * buffer) {
	float pickupMaterial = buffer[0];
	float mroofAngle = buffer[1];
	float mroofPos = buffer[2];
	Serial.println("StudentsRobot::PickOrder Received");

	myCommandsStatus = Waiting_for_approval_to_pickup;
}
