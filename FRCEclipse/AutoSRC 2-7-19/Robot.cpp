#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <cmath>
//Main header files.
#include <Robot.h>
#include "WPILib.h"
//Peripheral header files.
#include "Doublesolenoid.h"
#include "Timer.h"
#include <AnalogGyro.h>
#include <cmath>
#include <ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

using namespace std;

//State checkers to ensure various proper operation for toggling.
bool isHighGear;
bool isInHighGear;
bool isAutoState;
//State checkers to ensure proper toggling operation for pneumatic actuating.
//bool isActuator;
//bool isArmIn;
//Booleans for toggle state testing.
bool shiftingSoloTest;
//bool actuatorSoloTest;
bool autoSoloTest;

bool targetReached;

bool doneDriving;
bool doneTurning;

bool primaryInternalCheck;
bool secondaryInternalCheck;
bool NAVXResetCheck;
bool NAVXSoloTest;

int robotPos;

int switchPos;

int driveState;

int baseLineDelay;

//Gyroscope output of the AHRS NAVX board.
double navxGyro;
//Averaged value of both gyroscopes.
double robotThrottle;
double robotSteer;
double elevatorThrottle;
//Variable for use in the TURN_TO_ANGLE function as the setpoint.
double intakeThrottle;
//Variable for use in the TURN_TO_ANGLE function as the setpoint.
double autonomousAngleSet;
//Variable for use in the DRIVE_TO_DISTANCE function as the setpoint.
double autonomousDistanceSet;
//Wait time to allow air to flow through the system and shift the gears.
const float valveWait = 0.125;

const float gearRatio = 1.0/15.0;

const double motorVelocity = 0.5;

const double autonomousDistance[3] = {5.0,
		5.0};
const double autonomousRotation[1] = {
		270.0
		};

struct Leg_Switch_Data{
	double SWITCH_DIST;
	double SWITCH_ANGLE;
}switchSegments[3];

TalonSRX *leftMasterMotor;
TalonSRX *leftSlaveMotor;
TalonSRX *rightMasterMotor;
TalonSRX *rightSlaveMotor;

Joystick *leftJoystick;
Joystick *rightJoystick;

DoubleSolenoid *gearBox;

AHRS *NAVXBoard;

Timer *autonomousTimer;
void Robot::RobotInit() {

	isHighGear = false;
//	isActuator = false;
//
//	shiftingSoloTest = true;
//	actuatorSoloTest = true;

	targetReached = false;

	doneDriving = false;
	doneTurning = false;

	primaryInternalCheck = true;
	NAVXResetCheck = false;
	NAVXSoloTest = false;

	doneTurning = false;
	doneDriving = false;

	navxGyro = 0;

	robotThrottle = 0 ;
	elevatorThrottle = 0;
	intakeThrottle = 0;
	robotSteer = 0;

	leftMasterMotor = new TalonSRX(1);
	leftSlaveMotor = new TalonSRX(2);
	rightMasterMotor = new TalonSRX(3);
	rightSlaveMotor = new TalonSRX(4);

	gearBox = new DoubleSolenoid(2, 1);

    try {
        NAVXBoard = new AHRS(SPI::Port::kMXP);
    } catch (exception NAVXFailure ) {
        string NAVXErrorString = "Error instantiating NAVXBoard!";
        NAVXErrorString += NAVXFailure.what();
        DriverStation::ReportError(NAVXErrorString.c_str());
    }

	autonomousTimer = new Timer();

	//INITIAL SETPOINTS, CALIB, ETC.
	NAVXBoard->Reset();
	autonomousTimer->Reset();

	//ControlMode::kmode is now used to determing control method.
	//PercentOutput throttles Talons along -1/1 range.
	//Follower sets a Talons as a "slave" relative to another.
	leftMasterMotor->Set(ControlMode::PercentOutput, 0);
	rightMasterMotor->Set(ControlMode::PercentOutput, 0);

	//The follower ID must be correct.
	leftSlaveMotor->Set(ControlMode::Follower, 1);
	rightSlaveMotor->Set(ControlMode::Follower, 3);

	//TalonSRX PID parameter setup.
	//The pidxid of the sensor can be found in the web-configuration page of the Roborio.
	//Should be a 0, multiple id values not supported (yet).
	//Selects the sensor type and the channel the sensor is communicating on.
	leftMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	//Sets up the sensor type, the sensor ID number, DEFINED AS pidIdx, and timeout period.
	//Sets the read direction on the Encoders, with Clockwise being positive.
	leftMasterMotor->SetSensorPhase(true);
	//Sets up the various PIDF values to tune the motor output.

	rightMasterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	rightMasterMotor->SetSensorPhase(false);

	leftMasterMotor->SetInverted(false);
	leftSlaveMotor->SetInverted(false);
	rightMasterMotor->SetInverted(false);
	rightSlaveMotor->SetInverted(false);

	leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
	rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
}

bool DRIVE_TO_DISTANCE(double basePositionTarget) {
	const int tolerance = 1000;
	const int circumference = 6.000;
	const int driveRatio = 1.0/15.0;
	const int encoderRotTick = 4096.0;
	const int pi = 3.1415;

	double finalPositionTarget = 4096 * (((basePositionTarget * 12.0) - 4.0)/circumference);

	if(leftMasterMotor->GetSelectedSensorPosition(1) < finalPositionTarget) {
//		leftMasterMotor->Set(ControlMode::MotionMagic, finalPositionTarget);
//		rightMasterMotor->Set(ControlMode::MotionMagic, finalPositionTarget);
		leftMasterMotor->Set(ControlMode::PercentOutput, 0.5);
		rightMasterMotor->Set(ControlMode::PercentOutput, -0.5);
		secondaryInternalCheck = false;
		doneDriving = false;
	} else if (leftMasterMotor->GetSelectedSensorPosition(1) >= finalPositionTarget) {
		leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		secondaryInternalCheck = true;
		doneDriving = true;
	}

	return secondaryInternalCheck;
}

bool TURN_TO_ANGLE(double gyroValue) {
	const int tolerance = 7.500;
	double float lastGyroSetValue = 0;

	if((gyroValue <= (lastGyroSetValue - 7.5)) || (gyroValue >= (lastGyroSetValue + 7.5))) {
		if(gyroValue < lastGyroSetValue) {
			leftMasterMotor->Set(ControlMode::PercentOutput, 1.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, -1.0);
			primaryInternalCheck = false;
			doneTurning = false;
		} else if (gyroValue > lastGyroSetValue) {
			leftMasterMotor->Set(ControlMode::PercentOutput, -1.0);
			rightMasterMotor->Set(ControlMode::PercentOutput, 1.0);
			primaryInternalCheck = false;
			doneTurning = false;
		}
	} else if((gyroValue >= (lastGyroSetValue - 7.5)) || (gyroValue <= (lastGyroSetValue + 7.5))) {
		leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		primaryInternalCheck = true;
		doneTurning = false;
	}

	return primaryInternalCheck;
}
void Robot::AutonomousInit() {
	driveState = 0;
	doneDriving = false;
	doneTurning = false;
	NAVXBoard->Reset();
	switchSegments[0].SWITCH_DIST = 0.0;
	switchSegments[0].SWITCH_ANGLE = 1.0;
	switchSegments[1].SWITCH_DIST = 0.0;
	switchSegments[1].SWITCH_ANGLE = 1.0;
//	switchSegments[2].SWITCH_DIST = 0.0;
//	switchSegments[2].SWITCH_ANGLE = 1.0;
}

void Robot::AutonomousPeriodic() {
	if(driveState <= 1) {
		if(!doneDriving && !doneTurning) {
			doneDriving = DRIVE_TO_DISTANCE(switchSegments[1].SWITCH_DIST);
		} else if(doneDriving && !doneTurning) {
			doneTurning = TURN_TO_ANGLE(switchSegments[1].SWITCH_ANGLE);
		} else if(doneDriving && doneTurning) {
			driveState ++;
			leftMasterMotor->SetSelectedSensorPosition(0, 0, 10);
			rightMasterMotor->SetSelectedSensorPosition(0, 0, 10);
			NAVXBoard->Reset();
			doneDriving = false;
			doneTurning = false;
		}
	} else if (driveState > 1) {
		leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);
	}
}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
