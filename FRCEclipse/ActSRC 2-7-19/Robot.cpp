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

bool isActuatorUpright;
bool actuatorSoloTest;

double hatchThrottle;
double ballThrottle;

Joystick *leftJoystick;
Joystick *rightJoystick;

TalonSRX *actuatorMotor;
TalonSRX *hatchRoller;
TalonSRX *ballRoller;


void Robot::RobotInit() {
	isActuatorUpright = false;
	actuatorSoloTest = false;

	hatchThrottle = 0;
	ballThrottle = 0;

	actuatorMotor = new TalonSRX(7);
	hatchRoller = new TalonSRX(8);
	ballRoller = new TalonSRX(9);

	leftJoystick = new Joystick(1);
	rightJoystick = new Joystick(2);

	actuatorMotor->Config_kP(1, 1, 10);
	actuatorMotor->Config_kI(1, 1, 10);
	actuatorMotor->Config_kD(1, 1, 10);

	actuatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
	actuatorMotor->SetSensorPhase(true);

	actuatorMotor->Set(ControlMode::PercentOutput, 0);
	hatchRoller->Set(ControlMode::PercentOutput, 0);
	ballRoller->Set(ControlMode::PercentOutput, 0);

}

void ROTATE_UP() {
	actuatorSoloTest =! actuatorSoloTest;
	isActuatorUpright = true;
	SmartDashboard::PutBoolean("Actuator Position", isActuatorUpright);
}

void ROTATE_DOWN() {
	actuatorSoloTest =! actuatorSoloTest;
	isActuatorUpright = false;
	SmartDashboard::PutBoolean("Actuator Position", isActuatorUpright);
}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	if(rightJoystick->GetRawButton(3) && actuatorSoloTest) {
		if(isActuatorUpright) {
			ROTATE_UP();
			SmartDashboard::PutBoolean("Actuator Position", isActuatorUpright);
		} else if(!isActuatorUpright) {
			ROTATE_DOWN();
			SmartDashboard::PutBoolean("Actuator Position", isActuatorUpright);
		}
		actuatorSoloTest = false;
	}

	if(!leftJoystick->GetRawButton(1)) {
		actuatorSoloTest = true;
	}

	if(isActuatorUpright) {
		actuatorMotor->Set(ControlMode::Position, 4096.0/4);
	} else if(!isActuatorUpright) {
		actuatorMotor->Set(ControlMode::Position, 4096.0/2);
	}

	if(rightJoystick->GetRawButton(5)) {
		hatchThrottle = 1.0;
	} else {
		hatchThrottle = 0.0;
	}
	hatchRoller->Set(ControlMode::PercentOutput, hatchThrottle);

	if(rightJoystick->GetRawButton(6)) {
		ballThrottle = 1.0;
	} else {
		ballThrottle = 0.0;
	}
	hatchRoller->Set(ControlMode::PercentOutput, hatchThrottle);


}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
