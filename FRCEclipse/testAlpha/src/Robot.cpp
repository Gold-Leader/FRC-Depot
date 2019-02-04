#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
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

TalonSRX *alphaMotor;
TalonSRX *betaMotor;
TalonSRX *deltaMotor;
TalonSRX *epsilonMotor;

TalonSRX *elevatorAlpha;
TalonSRX *elevatorBeta;

Joystick *alphaStick;
Joystick *betaStick;

double throttle;
double steer;

double elevatorThrottle;

void Robot::RobotInit() {
	alphaMotor = new TalonSRX(1);
	betaMotor = new TalonSRX(3);
	deltaMotor = new TalonSRX(2);
	epsilonMotor = new TalonSRX(4);

	elevatorAlpha = new TalonSRX(5);
	elevatorBeta = new TalonSRX(6);


	alphaStick = new Joystick(1);
	betaStick = new Joystick(2);

	betaMotor->Set(ControlMode::Follower, 1);
	epsilonMotor->Set(ControlMode::Follower, 3);

	elevatorBeta->Set(ControlMode::Follower, 5);

	alphaMotor->Set(ControlMode::PercentOutput, 0);
	deltaMotor->Set(ControlMode::PercentOutput, 0);

	elevatorAlpha->Set(ControlMode::PercentOutput, 0);


	throttle = 0;
	steer = 0;

	elevatorThrottle = 0;
}

void Robot::AutonomousInit() {


}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	throttle = alphaStick->GetY();
	steer = alphaStick->GetX();

	alphaMotor->Set(ControlMode::PercentOutput, throttle - steer);
	deltaMotor->Set(ControlMode::PercentOutput, -throttle - steer);

	elevatorThrottle = betaStick->GetY();

	elevatorAlpha->Set(ControlMode::PercentOutput, elevatorThrottle);
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot)
