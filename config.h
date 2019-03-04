 /*
 * config.h
 *
 *  Created on: Nov 5, 2018
 *      Author: hephaestus
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define TEAM_NAME "Team2"

#define USE_WIFI

//
#define WHEEL_TRACK 260
#define WHEEL_RADIUS (3.165*25.4/2.0)
//Line Following
#define NUM_LINES_LANE 4
#define NUM_LINES_CROSS 1
#define TURN_ANGLE 90
#define OFFSET = -10;

//PID vals
#define KP_PLASTIC 0.001
#define KI_PLASTIC 0.0001
#define KD_PLASTIC 0.9
#define KP_ALUM 0.01
#define KI_ALUM 0.0001
#define KD_ALUM 0.6

//Materials
#define ALUMINUM 1.00
#define PLASTIC 0.00
// Pins

/**
 * Drive motor 1 Servo PWM pin
 */
#define MOTOR1_PWM 15
/**
 * Drive motor 2 Servo PWM pin
 */
#define MOTOR2_PWM 4
/**
 * Drive motor 3 10Khz full duty PWM pin
 */
#define MOTOR3_PWM 12
/**
 * Pin for setting the direction of the H-Bridge
 */
#define MOTOR3_DIR 26
#define MOTOR3_ENABLE_PIN 13

//Encoder pins
#define MOTOR1_ENCA 18
#define MOTOR1_ENCB 19

#define MOTOR2_ENCA 17
#define MOTOR2_ENCB 16

#define MOTOR3_ENCA 27
#define MOTOR3_ENCB 14

#define ENCODER_POS_45 1850.0
#define ENCODER_POS_25 3580.0
#define ENCODER_POS_MAX 3400.0
// Line Sensor Pins
#define LINE_SENSE_ONE 			36
#define LINE_SENSE_TWO 			39
#define EMITTER_PIN         34  // emitter is controlled by digital pin



/**
 * Gripper pin for Servo
 */
#define SERVO_PIN 5
#define GRABBER_OPEN 110
#define GRABBER_CLOSED 180

#endif /* SRC_CONFIG_H_ */
