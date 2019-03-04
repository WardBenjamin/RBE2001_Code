

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

chassis = new DrivingChassis(motor1, motor2, WHEEL_TRACK, WHEEL_RADIUS);
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
 servo->write(GRABBER_CLOSED);
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
	Serial.println("State Machine Startup");

	myCommandsStatus = Ready_for_new_task;
    digitalWrite(EMITTER_PIN, HIGH); // turn on IR LEDs
    digitalWrite(MOTOR3_ENABLE_PIN, HIGH); // enable HBridge

    lastStatus = StartupRobot;
    nextStatus = PickupPanelStart;
    status = WAIT_FOR_PICKORDER; // wait for control station info
	break;

	case StartRunning:
  myCommandsStatus = Heading_to_pickup;
  //servo->write(GRABBER_OPEN);
  
  float val;
  if(roofAngle == 45){
   val = ENCODER_POS_45; 
  }else if(roofAngle == 25){
    val = ENCODER_POS_25; 
  }
  lineFollower->lineCount = 0;
  lineFollower->NUM_LINES_LANE / roofPos;
  motor3->startInterpolationDegrees(val, 5000, SIN);
  nextStatus = LF_Forward4_Init;
  status = WAIT_FOR_MOTORS_TO_FINISH;
  
  break;

	case PlacePanel:
		myCommandsStatus = Waiting_for_approval_to_dropoff;

		//determine appropriate angle to dropoff panel
		float placeAngle;
		if(roofAngle == 45){
			placeAngle = ENCODER_POS_45;
		}else if(roofAngle == 25){
			placeAngle = ENCODER_POS_25;
		}
		//move grabber to correct position
		motor3->startInterpolationDegrees(placeAngle, 2000, SIN);
		nextStatus = Transition_Post_Dropoff;
		lastStatus = PlacePanel;
		status = WAIT_FOR_APPROVE; //wait for approval to release
		break;
	case PlacePanelFinal:
		myCommandsStatus = Dropping_off;
		chassis->driveForward(-75, 1000); // backup as panel goes down to have correct spacing
		motor3->startInterpolationDegrees(0, 5000, SIN); //slowly place panel down
		nextStatus = Halting;
		lastStatus = PlacePanelFinal;
		status = WAIT_FOR_MOTORS_TO_FINISH;
		break;
  
	case PickupPanel:
		myCommandsStatus = Waiting_for_approval_to_pickup;
		chassis->driveForward(75, 1000); // drive forward from line into correct position
		nextStatus = Transition_Post_Pickup;
		lastStatus = PickupPanel;
		status = WAIT_FOR_APPROVE; //wait for operator approval
		break;
	case Transition_Post_Dropoff:
		myCommandsStatus = Dropping_off;
		servo->write(GRABBER_OPEN); // release panel
		numLines=NUM_LINES_RETURN_DROPOFF; //adjust line target for return
		lineFollower->lineCount = 0; //reset line counter
		motor3->startInterpolationDegrees(ENCODER_POS_MAX, 5000, SIN); //move grabber off of panel
		chassis->driveForward(-50,1000); //backup from line
		nextStatus = LF_Backup2_Init; //begin backup
		lastStatus = Transition_Post_Dropoff;
		status = WAIT_FOR_MOTORS_TO_FINISH;
		break;
	case Transition_Post_Pickup:
		myCommandsStatus = Heading_to_safe_zone;
		servo->write(GRABBER_CLOSED); //grip panel
		lineFollower->lineCount = 0; //reset line counter
		lineFollower->numLines =NUM_LINES_LANE;  //adjust line target for return

		//pick up panel
		motor3->startInterpolationDegrees(ENCODER_POS_MAX, 5000, SIN);
		nextStatus = LF_Backup3_Init;
		lastStatus = Transition_Post_Pickup;
		status = WAIT_FOR_MOTORS_TO_FINISH;
		break;
	case PickupPanelStart:
		//lift panel slowly
	  motor3->startInterpolationDegrees(ENCODER_POS_MAX, 5000, SIN)
	  lastStatus = PickupPanelStart;
	  nextStatus = LF_Backup_Init;
	  status = WAIT_FOR_MOTORS_TO_FINISH;
	  break;
  
	/* The following two states work in conjunction with one another.
	LF_Backup_Init moves the robot backward slightly, and then transitions to
	LF_Backup_Detect, which checks for the current line following status, and then transitions back to
	LF_Backup_Init. The loop is broken once the correct number of horizontal lines have been detected
	Additionally, after a horizontal line is detected, the CountLineReverse state may be entered to avoid double-counting
	*/

	case LF_Backup_Init:
		Serial.println("State: Backup_Init");
		chassis->driveForward(-10, 0); // move backward slightly
		lastStatus = LF_Backup_Init;
		status = LF_Backup_Detect; //next, check line following status
    
		break;
  case LF_Backup_Detect:

    Serial.println("State: Backup_Detect");
    //Uncomment line below for debugging purposes
    //lineFollower->readSensors();

    //check for horizontal line
    if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val >= lineFollower->BLACK){
      lineCount++; //increment line counter

      if(roofPos == lineCount){ //check if current line is the right one
       nextStatus = LF_Transition_1; //begin the switch to crossing line
       status = STOP;
      }
      else{ //take appropriate action to resume looking for hoz. lines
        status = CountLineReverse;  
      }
    }
    else{

    //chassis tilted clockwise relative to line
    if(lineFollower->sensor1Val >= lineFollower->BLACK && lineFollower->sensor2Val <= lineFollower->WHITE){
    	//straighten out -- rotate left wheel backward
    	motor1->startInterpolationDegrees(motor1->getAngleDegrees() - 60, 1000, SIN);
    }
    //chassis tilted counterclockwise relative to line
    else if(lineFollower->sensor1Val <= lineFollower->WHITE && lineFollower->sensor2Val >= lineFollower->BLACK){
    	//straighten out -- rotate right wheel backward
    	motor2->startInterpolationDegrees(motor2->getAngleDegrees() - 60, 1000, SIN);
    }
    lastStatus = LF_Backup_Detect;
    nextStatus = LF_Backup_Init; //loop
    status = WAIT_FOR_MOTORS_TO_FINISH;  
    }
    break;
    
  case LF_Transition_2: //part 2 of the transition between backing up from staging area and dropoff
	  Serial.println("State: LF_Transition_2");
	  lineFollower->lineCount = 0; //reset line counter
	  lineFollower->numLines =NUM_LINES_LANE;//adjust lines target for roof lane
    chassis->driveForward(50, 1000); //scoot off of line
    nextStatus = LF_Forward_Init;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition_2;
    break;
  case LF_Transition_1: //part 1 of the transition between backing up from staging area and dropoff
    Serial.println("State: LF_Transition_1");
    chassis->turnDegrees(95, 4000); //turn to line up with roof lane line
    nextStatus = LF_Transition_2;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition_1;
    break;


case LF_Transition2_1: //part 1 of the transition between backing up from dropoff and navigating to pickup lane
  
    Serial.println("State: LF_Transition2_1");
    chassis->driveForward(15, 1000);
    nextStatus = LF_Transition2_2;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition2_1;
    break;
  case LF_Transition2_2: //part 2 of the transition between backing up from dropoff and navigating to pickup lane
  
    Serial.println("State: LF_Transition2_2");
    chassis->turnDegrees(90, 4000); // turns from current lane back onto crossing line
  //set number of lines opposite of pos value so robot goes to correct second spot
    lineFollower->numLines =1;
  if(roofPos == 1){
    lineFollower->numLines =2;
  }
    nextStatus = LF_Forward2_Init;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition2_2;
    break;

case LF_Transition3_1: //part 1 of the transition between moving forward from the crossing line and to pickup lane
    lineFollower->lineCount = 0; //reset line count
    Serial.println("State: LF_Transition3_1");
    chassis->driveForward(-10, 1000); //move VTC to line before turn
    nextStatus = LF_Transition3_2;
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition3_1;
    break;
  case LF_Transition3_2: //part 2 of the transition between moving forward from the crossing line and to pickup lane
  
    Serial.println("State: LF_Transition3_2");
    chassis->turnDegrees(90, 4000); //turn onto the pickup lane line
    numLines=NUM_LINES_LANE;
    nextStatus = LF_Forward4_Init;
   
    status = WAIT_FOR_MOTORS_TO_FINISH;
    lastStatus = LF_Transition3_2;
    break;


    /* The following two states work in conjunction with one another.
    	LF_Forward_Init moves the robot forward slightly, and then transitions to
    	LF_Forward_Detect, which checks for the current line following status, and then transitions back to
    	LF_Forward_Init. The loop is broken once the correct number of horizontal lines have been detected
    	Additionally, after a horizontal line is detected, the CountLine state may be entered to avoid double-counting
    	*/
    
  case LF_Forward_Init: //state used by the robot to get from cross line to the correct position for collector placement
  
    Serial.println("State: LF_Forward_Init");
    followLineDrive(&status, &nextStatus,LF_Forward_Detect, WAIT_FOR_MOTORS_TO_FINISH,50);
    lastStatus = LF_Forward_Init;
    break;
  case LF_Forward_Detect:
	Serial.println("State: LF_Forward_Detect");
    lastStatus = LF_Forward_Detect; //state that checks and corrects line alignment until correct number of horizontal lines crossed
    //Uncomment below for debugging
    //lineFollower->readSensors();
    followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Forward_Init,
    			CountLine, LF_Transition_1,-60);
    break;
    case LF_Forward2_Init:
  
    Serial.println("State: LF_Forward2_Init");
    followLineDrive(&status, &nextStatus,LF_Forward2_Detect, WAIT_FOR_MOTORS_TO_FINISH,dirMult*50);
    lastStatus = LF_Forward2_Init;
    break;
  case LF_Forward2_Detect:
    lastStatus = LF_Forward2_Detect;
    Serial.println("State: LF_Forward2_Detect");
    followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Forward2_Init,
    			CountLine, LF_Transition3_1,-60);
    break;



  case LF_Forward3_Init:
  
    Serial.println("State: LF_Forward_Init");
    followLineDrive(&status, &nextStatus,LF_Forward3_Detect, WAIT_FOR_MOTORS_TO_FINISH,25);
    lastStatus = LF_Forward3_Init;
    break;
  case LF_Forward3_Detect:
    lastStatus = LF_Forward3_Detect;
    Serial.println("State: LF_Forward3_Detect");
    lineFollower->readSensors();
    followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Forward3_Init,
    			CountLinee, PickupPanel,-60);

case LF_Backup3_Init:
    lastStatus = LF_Backup3_Init;
    Serial.println("State: Backup3_Init");
    followLineDrive(&status, &nextStatus,LF_Backup3_Detect, WAIT_FOR_MOTORS_TO_FINISH,-10);
    
    break;
  case LF_Backup3_Detect:
  lastStatus = LF_Backup3_Detect;
    Serial.println("State: Backup_Detect");
    lineFollower->readSensors();
    followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Backup3_Init,
    			CountLineReverse, LF_Transition3_1,-60);
    break;


case LF_Forward4_Init:
  
    Serial.println("State: LF_Forward4_Init");
    followLineDrive(&status, &nextStatus,LF_Forward4_Detect, WAIT_FOR_MOTORS_TO_FINISH,25);
    lastStatus = LF_Forward4_Init;
    break;
  case LF_Forward4_Detect:
	  followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Forward4_Init,
	  			CountLine, PlacePanelFinal, -60);    break;

    
   case CountLine:
   motor1->stop();
   motor2->stop();
    Serial.printf("LINE COUNT %d\n", lineCount);
    //lastStatus = CountLine;
    chassis->driveForward(25, 1000);
    //nextStatus = LF_Forward_Detect;
    nextTime=millis()+2000;
    status = WAIT_FOR_TIME;
    lastStatus = CountLine;
   break;
   case CountLineReverse:
   motor1->stop();
   motor2->stop();
    //Serial.printf("LINE COUNT %d\n", lineCount);
    //lastStatus = CountLine;
    chassis->driveForward(-25, 1000);
    //nextStatus = LF_Forward_Detect;
    nextTime=millis()+2000;
    //status = WAIT_FOR_MOTORS_TO_FINISH;
    status = WAIT_FOR_TIME;
    lastStatus = CountLineReverse;
   break;
   case LF_Backup2_Init:
    lastStatus = LF_Backup2_Init;
    Serial.println("State: Backup2_Init");
    followLineDrive(&status, &nextStatus,LF_Backup2_Detect, WAIT_FOR_MOTORS_TO_FINISH,-50);
    
   break;
  case LF_Backup2_Detect:
  lastStatus = LF_Backup2_Detect;
    Serial.println("State: Backup_Detect");
    lineFollower->readSensors();
    followLineDetect(&status, &nextStatus, WAIT_FOR_MOTORS_TO_FINISH, LF_Backup2_Init,
			CountLineReverse, LF_Transition2_1,-60);
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
   
  case WAIT_FOR_PICKORDER:
  //Serial.println("State: WAIT_FOR_MOTORS_TO_FINISH");
  //Serial.println(motor3->getAngleDegrees());
    if (pickorder) {
      pickorder = false;
      status = nextStatus;
    }
    lastStatus = WAIT_FOR_APPROVE;
    break;
  case STOP:
    Serial.println("State: STOP");
    motor3->stop();
    motor2->stop();
    motor1->stop();
    status = nextStatus;
    lastStatus = STOP;
    break;
	case Halting:
  servo->write(GRABBER_OPEN);
		myCommandsStatus = Fault_obstructed_path;
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
	/*if (myCommandsStatus == Waiting_for_approval_to_pickup) {
		myCommandsStatus = Waiting_for_approval_to_dropoff;
	} else {
		myCommandsStatus = Ready_for_new_task;
	}*/
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
	//myCommandsStatus = Ready_for_new_task;
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
	pickorder = true;
 
	material = buffer[0];
	roofAngle = buffer[1];
	roofPos = buffer[2];


  if(roofPos == 1){
    dirMult = -1;
  }else{
    dirMult = 1;
  }
  
  if(material == ALUMINUM){
    motor3->SetTunings(KP_ALUM, KI_ALUM, KD_ALUM);
  }
  else if(material == PLASTIC){
    motor3->SetTunings(KP_PLASTIC, KI_PLASTIC, KD_PLASTIC);
  }
	//myCommandsStatus = Waiting_for_approval_to_pickup;
}
