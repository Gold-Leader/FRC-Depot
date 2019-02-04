#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <cmath>
//Main header files.
#include <Robot.h>
#include "frc/WPILib.h"
//Peripheral header files.
#include "frc/Doublesolenoid.h"
#include "frc/Timer.h"
#include <frc/AnalogGyro.h>
#include <cmath>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

using namespace std;

//State checkers to ensure various proper operation for toggling.
bool isHighGear;
bool isInHighGear;
//State checkers to ensure proper toggling operation for pneumatic actuating.
bool isActuator;
bool isArmIn;
//Booleans for toggle state testing.
bool shiftingSoloTest;
bool actuatorSoloTest;

bool targetReached;

bool doneDriving;
bool doneTurning;

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

TalonSRX *leftMasterMotor;
TalonSRX *leftSlaveMotor;
TalonSRX *rightMasterMotor;
TalonSRX *rightSlaveMotor;

Joystick *driveStick;
Joystick *auxStick;

DoubleSolenoid *gearBox;

AHRS *NAVXBoard;

Timer *autonomousTimer;
void Robot::RobotInit() {

	isHighGear = false;
	isActuator = false;

	shiftingSoloTest = true;
	actuatorSoloTest = true;

	targetReached = false;

	doneDriving = false;
	doneTurning = false;

	navxGyro = 0;

	robotThrottle = 0;
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
bool TURN_TO_ANGLE (int autonomousAngleSet) {

	navxGyro = NAVXBoard->GetAngle();

	const float motorRotationVel = 0.375;
	const float tolerance = 9.0;

	if(((navxGyro <= autonomousAngleSet)) && !targetReached) {
		leftMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
		rightMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
		doneTurning = false;
		SmartDashboard::PutString("Motion State", "Turning Left!");
	} else if(((navxGyro >= autonomousAngleSet)) && !targetReached) {
		leftMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
		rightMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
		doneTurning = false;
		SmartDashboard::PutString("Motion State", "Turning Right!");
	} else if((navxGyro > autonomousAngleSet - tolerance) && (
			navxGyro < autonomousAngleSet + tolerance)) {
		targetReached = true;
	}

	if(targetReached){
		leftMasterMotor->Set(ControlMode::PercentOutput, (0));
		rightMasterMotor->Set(ControlMode::PercentOutput, (0));
		doneTurning = true;
		SmartDashboard::PutString("Motion State", "Done Turning!");
		targetReached = true;
	}

	return doneTurning;
}
//bool TURN_TO_ANGLE (int autonomousAngleSet) {
//
//	navxGyro = NAVXBoard->GetAngle();
//
//	double tolerance = 9.0;
//
//	//All positive motor values will rotate robot left.
//	//Inversely, all negative will rotate robot right.
//	if((!targetReached && (navxGyro < autonomousAngleSet))) {
//		leftMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
//		rightMasterMotor->Set(ControlMode::PercentOutput, (-motorVelocity));
//		doneTurning = false;
//		if((navxGyro > autonomousAngleSet - tolerance) && (
//				navxGyro < autonomousAngleSet + tolerance)) {
//			targetReached = true;
//		}
//		SmartDashboard::PutString("Motion State", "Turning Left!");
//
//	} else if(!targetReached && ((navxGyro > autonomousAngleSet))) {
//		leftMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
//		rightMasterMotor->Set(ControlMode::PercentOutput, (motorVelocity));
//		doneTurning = false;
//		if((navxGyro > autonomousAngleSet - tolerance) &&
//				(navxGyro< autonomousAngleSet + tolerance)) {
//			targetReached = true;
//		}
//		SmartDashboard::PutString("Motion State", "Turning Right!");
//
//	}
//	if(targetReached){
//		leftMasterMotor->Set(ControlMode::PercentOutput, (0));
//		rightMasterMotor->Set(ControlMode::PercentOutput, (0));
//		doneTurning = true;
//		SmartDashboard::PutString("Motion State", "Done Turning!");
//		targetReached = true;
//	}
//	return doneTurning;
//}
// void DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
// 	//Proprietary unit for each Encoder rotation, 4096 units per rotation.
// 	const float encoderRotTick = 4096.0;
// 	const float PI = 3.1415;
// 	//Calculation of circumference, used for autonomous driving control.
// 	const float circumference = (PI*6.0);
// }
bool DRIVE_TO_DISTANCE(int autonomousDistanceSet) {
	//Proprietary unit for each Encoder rotation, 4096 units per rotation.
	const float encoderRotTick = 4096.0;
	const float PI = 3.1415;
	//Calculation of circumference, used for autonomous driving control.
	const float circumference = (PI*6.0);

	if((((gearRatio) * leftMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick) &&
			(((gearRatio) * rightMasterMotor->GetSelectedSensorPosition(0)) < (((autonomousDistanceSet - 4.0)/(circumference)) * encoderRotTick)))) {

		leftMasterMotor->Set(ControlMode::PercentOutput, -motorVelocity);
		rightMasterMotor->Set(ControlMode::PercentOutput, motorVelocity);

		SmartDashboard::PutString("Motion State", "Currently Moving Forward!");
		//When doneDriving is false, should continue driving along.
		doneDriving = false;
	} else {
		leftMasterMotor->Set(ControlMode::PercentOutput, 0.0);
		rightMasterMotor->Set(ControlMode::PercentOutput, 0.0);

		SmartDashboard::PutString("Motion State", "Done Forward!");
		//When doneDrving is true, should schedule the next segment.
		doneDriving = true;
	}
	return doneDriving;
}
void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	if(!doneTurning && driveStick->GetRawButton(3)) {
		TURN_TO_ANGLE(90);
	} else if(doneTurning) {

	}
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
