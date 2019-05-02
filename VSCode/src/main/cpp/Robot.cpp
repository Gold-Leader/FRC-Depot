#include <iostream>
#include <string>
#include <memory>
#include <stdlib.h>
#include <math.h>
//Main header files.
#include <Robot.h>
#include "frc/WPILib.h"
//Peripheral header files.
#include "frc/Doublesolenoid.h"
#include "frc/Timer.h"
#include <frc/AnalogGyro.h>
#include <frc/ADXRS450_Gyro.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <AHRS.h>
//Vision header files
#include "cameraserver/CameraServer.h"
#include "vision/VisionPipeline.h"
#include "vision/VisionRunner.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "C:\Users\GoldA\GRIP\GripPipeline.h"
#include "C:\Users\GoldA\GRIP\GripPipeline.cpp"

//Using assets from:
//C:\Users\GoldA\Documents\FRCEclipse
//C:\Users\GoldA\GRIP\GripPipeline.h
//C:\Users\GoldA\GRIP\GripPipeline.cpp
//autonomousMobility.cpp
//intakeMobility.cpp
//linearSlideMobility.cpp
//visionExperiment.cpp


using namespace frc;
using namespace std;
using namespace cs;
using namespace grip;
using namespace rev;

    const int imageHeight = 480;
    const int imageWidth = 640;
    //LOOK HERE!!!!!
    //Fix this later.
    const int hatchOffsetDistance = 1043.037835 * 28.0;
    const int hatchInitialDistiance = 1043.037835 * 19.0;
    const int portOffsetDistance = 1043.037835 * 28.0;
    const int portInitiallDistance = 1043.037835 * 27.5;
    const double turnScaleDown = 0.5;

    enum leftJoystickMap{
        TEMP_HATCH_PLACE = 10, //Or left Toggle.
        TEMP_ELEVATOR_CONTROL_TOGGLE = 11, //Or right top toggle.
        TEMP_ACTUATOR_CONTROL_TOGGLE = 12, //Or right bottom toggle.
		TEMP_ACTUATOR_POV_UP = 180,
		TEMP_ACTUATOR_POV_DOWN = 0,
        TEMP_ACTUATOR_MANAUL_UP = 6,
        TEMP_ACTUATOR_MANAUL_DOWN = 4,
        TEMP_ELEVATOR_STATE_UP = 5,
        TEMP_ELEVATOR_STATE_DOWN = 3,
    };

	enum rightJoystickMap{
        GEAR_CHANGE = 2,
	};
    enum GamepadMap{
        HATCH_PLACE = 10, //Or left Toggle.
        ELEVATOR_CONTROL_TOGGLE = 11, //Or right top toggle.
        ACTUATOR_CONTROL_TOGGLE = 12, //Or right bottom toggle.
		ACTUATOR_POV_UP = 180,
		ACTUATOR_POV_DOWN = 0,
        ACTUATOR_MANAUL_UP = 6,
        ACTUATOR_MANAUL_DOWN = 4,
        ELEVATOR_STATE_UP = 5,
        ELEVATOR_STATE_DOWN = 3,
	};

    const double elevatorData[6] = {
        (hatchInitialDistiance),
		(portInitiallDistance),
		(hatchInitialDistiance) + (hatchOffsetDistance),
		(portInitiallDistance) + (portOffsetDistance),
		(hatchInitialDistiance) + (hatchOffsetDistance * 2),
		(portInitiallDistance) + (portOffsetDistance * 2),
    };
    //LOOK HERE!!!!!
    //Fix this later.
	double actuatorData[2] = {
		4096.0/4.0,
		4096.0/2.0
	};

	double robotThrottle;
	double robotSteer;
    double elevatorThrottle;
    double elevatorTarget;

    double actuatorThrottle;
    double actuatorTarget;
    double hatchThrottle;
    double ballThrottle;

    double centerCountoursX;
    double processedImageWidth;

    int elevatorState;
    int actuatorState;

    //Common control/toggling blocks.
    //is[mechanism] controls state
    //isIn[mechanism] Dashboard display state
    //[mechanism]SoloTest manages a solo activation of block
    bool isHighGear;
    bool isInHighGear;
    bool shiftingSoloTest;

    bool isHatchReady;
    bool isHatchIntakeReady;

    bool isHOverride;
    bool isHatchOverride;
    bool hatchOverrideSoloTest;

    bool isEOverride;
    bool isElevatorOverride;
    bool elevatorOverrideSoloTest;
    bool elevatorRaiseSoloTest;
    bool elevatorLowerSoloTest;

    bool isAOverride;
    bool isActuatorOverride;
    bool actuatorOverrideSoloTest;
    bool actuatorFWDPivotSoloTest;
    bool actuatorRWSPivotSoloTest;

    thread *visionThreadProcess;
    mutex *imageLock;

    Joystick *leftJoystick;
    Joystick *rightJoystick;
	Joystick *gamePad;

	DoubleSolenoid *gearBox;
	DoubleSolenoid *hatchDisconnect;
    DoubleSolenoid *hatchIntakeOutake;

    CANSparkMax *leftMasterMotor;
    CANSparkMax *leftSlaveMotor;
    CANSparkMax *rightMasterMotor;
    CANSparkMax *rightSlaveMotor;


    // TalonSRX *leftMasterMotor;
    // TalonSRX *leftSlaveMotor;
    // TalonSRX *rightMasterMotor;
    // TalonSRX *rightSlaveMotor;
	TalonSRX *elevatorMasterMotor;
    TalonSRX *elevatorSlaveMotor;
	TalonSRX *actuatorMotor;
    TalonSRX *hatchRoller;
    TalonSRX *ballRoller;
	//Unallocated

    frc::DriverStation *DSController;

    AHRS *NAVXBoard;
    GripPipeline *GRIPPRocess;
    VisionRunner<grip::GripPipeline> *visionProcess;

void Robot::RobotInit() {
	robotThrottle = 0 ;
	robotSteer = 0;
    elevatorThrottle = 0;

    actuatorThrottle = 0;
    hatchThrottle = 0;
    ballThrottle = 0;

    centerCountoursX = 0;

    elevatorState = 0;
    actuatorState = 0;

    //Alawys init [mechanism]SoloTest to TRUE;
    isHighGear = false;
    isInHighGear = false;
    shiftingSoloTest = true;

    isHatchReady = true;
    isHatchIntakeReady = true;


    isHOverride = false;
    isHatchOverride = false;
    hatchOverrideSoloTest = true;

    isEOverride = false;
    isElevatorOverride = false;
    elevatorOverrideSoloTest = true;
    elevatorRaiseSoloTest = true;
    elevatorLowerSoloTest = true;

    isAOverride = false;
	isActuatorOverride = true;
	actuatorOverrideSoloTest = false;
	actuatorFWDPivotSoloTest = true;
	actuatorRWSPivotSoloTest = true;

    imageLock = new mutex();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    gamePad = new Joystick(2);

    gearBox = new DoubleSolenoid(2, 1);
    hatchDisconnect = new DoubleSolenoid(6, 7);
    hatchIntakeOutake = new DoubleSolenoid(4, 5);

    leftMasterMotor = new CANSparkMax(1, CANSparkMax::MotorType::kBrushless);
    leftSlaveMotor = new CANSparkMax(2, CANSparkMax::MotorType::kBrushless);
    rightMasterMotor = new CANSparkMax(3, CANSparkMax::MotorType::kBrushless);
    rightSlaveMotor = new CANSparkMax(4, CANSparkMax::MotorType::kBrushless);

	// leftMasterMotor = new TalonSRX(1);
	// leftSlaveMotor = new TalonSRX(2);
	// rightMasterMotor = new TalonSRX(3);
	// rightSlaveMotor = new TalonSRX(4);
	elevatorMasterMotor = new TalonSRX(5);
    elevatorSlaveMotor = new TalonSRX(7);
    hatchRoller = new TalonSRX(6);
    ballRoller = new TalonSRX(8);
    actuatorMotor = new TalonSRX(9);
	//Unallocated

    elevatorSlaveMotor->Set(ControlMode::Follower, 5);

    leftMasterMotor->SetInverted(true);
	leftSlaveMotor->SetInverted(true);
	rightMasterMotor->SetInverted(true);
	rightSlaveMotor->SetInverted(true);

    elevatorMasterMotor->SetInverted(false);
    elevatorSlaveMotor->SetInverted(false);

    actuatorMotor->SetInverted(false);

    elevatorMasterMotor->ConfigPeakOutputForward(0.25, 10);
    elevatorMasterMotor->ConfigPeakOutputReverse(-1.0, 10);
    elevatorSlaveMotor->ConfigPeakOutputForward(0.25, 10);
    elevatorSlaveMotor->ConfigPeakOutputReverse(-1.0, 10);

    actuatorMotor->ConfigPeakOutputForward(0.5, 10);
    actuatorMotor->ConfigPeakOutputReverse(-0.5, 10);

    //Ball out
    hatchRoller->ConfigPeakOutputForward(0.90, 10);
    //Ball in
    hatchRoller->ConfigPeakOutputReverse(-0.90, 10);

    elevatorMasterMotor->Config_kP(1, 1, 10);
    elevatorMasterMotor->Config_kI(1, 1, 10);
    elevatorMasterMotor->Config_kD(1, 1, 10);

	actuatorMotor->Config_kP(1, 1, 10);
	actuatorMotor->Config_kI(1, 1, 10);
	actuatorMotor->Config_kD(1, 1, 10);

    leftMasterMotor->Set(0.0);
	rightMasterMotor->Set(0.0);
    leftSlaveMotor->Set(0.0);
    rightSlaveMotor->Set(0.0);
    
	elevatorMasterMotor->Set(ControlMode::PercentOutput, 0);
	actuatorMotor->Set(ControlMode::PercentOutput, 0);
    hatchRoller->Set(ControlMode::PercentOutput, 0);
    ballRoller->Set(ControlMode::PercentOutput, 0);

    try {
        NAVXBoard = new AHRS(SPI::Port::kMXP);
    } catch (exception& NAVXFailure ) {
        string NAVXErrorString = "Error instantiating NAVXBoard!";
        NAVXErrorString += NAVXFailure.what();
        DriverStation::ReportError(NAVXErrorString.c_str());
    }

    DriverStation& DSController = DriverStation::GetInstance();

    cs::UsbCamera elevatorCamera = CameraServer::GetInstance()->StartAutomaticCapture();
    elevatorCamera.SetResolution(imageWidth, imageHeight);
    elevatorCamera.
    elevatorCamera.SetFPS(25);

}
void Robot::RobotPeriodic() {
    SmartDashboard::PutBoolean("Gearbox State", isInHighGear);
	SmartDashboard::PutBoolean("Hatch State", isHatchReady);
    SmartDashboard::PutBoolean("Hatch Intake State", isHatchIntakeReady);
    // SmartDashboard::PutBoolean("Actuator Override", isHatchOverride);
    // SmartDashboard::PutBoolean("Elevator Override", isElevatorOverride);
    // SmartDashboard::PutNumber("Current Elevator State", elevatorState + 1);
    // SmartDashboard::PutNumber("Current Elevator State", actuatorState + 1);
    // SmartDashboard::PutNumber("Current Elevator Target", elevatorData[elevatorState]);
    // SmartDashboard::PutNumber("Current Elevator Target", actuatorData[actuatorState]);
    SmartDashboard::PutString("Engine Status", "CHECK ENGINE!!!!!");
    SmartDashboard::PutNumber("Master ELevator Power", elevatorMasterMotor->GetOutputCurrent());
    SmartDashboard::PutNumber("Slave Elevator Power", elevatorSlaveMotor->GetOutputCurrent());
}

void SHIFT_HIGH () {
    //Remember to use DoubleSolenoid::Value::[value] not DoubleSolenoid::[value].
    //REMEMBER ANGEL!!!!!
    //And do not use a pulse method to toggle DoubleSolenoids, frc::Wait(double) causes control pauses.
    //Just push to one side and hold it there.
    //Please work and clap.
	gearBox->Set(frc::DoubleSolenoid::Value::kForward);
	isHighGear =! isHighGear;
	//Where green is high gear, and red is low gear.
	isInHighGear = false;
	SmartDashboard::PutBoolean("Gearbox State", isInHighGear);
}

void SHIFT_LOW () {
	gearBox->Set(frc::DoubleSolenoid::Value::kReverse);
	isHighGear =! isHighGear;
	isInHighGear = true;
	SmartDashboard::PutBoolean("Gearbox State", isInHighGear);
}

void Robot::AutonomousInit() {
    //Always init these to obvious-duh values.
    elevatorState = 0;
    isHatchReady = true;
    isElevatorOverride = false;
    isActuatorOverride = false;
    SHIFT_HIGH();
    hatchDisconnect->Set(DoubleSolenoid::Value::kReverse);
    hatchIntakeOutake->Set(DoubleSolenoid::Value::kReverse);
}

void Robot::AutonomousPeriodic() {

    //Drive control block.
    //Implements a simple threshold limit for both steering and throttle values.
    //Motor power assignment below block.
    //And remember that GetY() refers to vertical stick movement, GetX() refers to horizontal.
	if(abs(rightJoystick->GetY()) < 0.05) {
		robotThrottle = 0;
	} else {
		robotThrottle = rightJoystick->GetY();
	}
	if(abs(rightJoystick->GetX()) < 0.05) {
		robotSteer = 0;
	} else {
		robotSteer =  (rightJoystick->GetX());
	}
    leftMasterMotor->Set(robotThrottle + (turnScaleDown*robotSteer));
	rightMasterMotor->Set(-(robotThrottle - (turnScaleDown*robotSteer)));
    leftSlaveMotor->Set(robotThrottle + (turnScaleDown*robotSteer));
    rightSlaveMotor->Set(-(robotThrottle - (turnScaleDown*robotSteer)));

    //Intake, ball and hatch, control block.
    if(leftJoystick->GetRawButton(5)) {
		hatchThrottle = 1.0;
	} else if(leftJoystick->GetRawButton(3)) {
		hatchThrottle = -1.0;
	} else {
		hatchThrottle = 0.0;
	}
	hatchRoller->Set(ControlMode::PercentOutput, hatchThrottle);
    
    //Transmission control block.
    //Implements a bool control system that flips a bool [mechanism]SoloTest instantly.
    //For all your single toggle needs!
    //Function-side code flips is[mechanism] and assigns is[mechanism]Ready bools for reuse and driver use.
    //Why am I writing literal paragraphs for these?
    if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
		if(isHighGear) {
			SHIFT_HIGH();
		} else if(!isHighGear) {
			SHIFT_LOW();
		}
		shiftingSoloTest = false;
	}
    //Once [mechanism]SoloTest flips in the block above, it awaits for the use to release the button.
    //Because [mechanism]SoloTest is checked in the first control loop and not in the second.
    //So, once the button is released shiftingSoloTest flips back ready for reuse.
	if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
		shiftingSoloTest = true;
	}
    
    //Two simple control blocks to manage the hatch piston.
    //Persistent set to kForward to keep the piston in. 
    if(leftJoystick->GetTrigger()) {
        isHatchReady = false;
    	SmartDashboard::PutBoolean("Hatch State", isHatchReady);
        hatchDisconnect->Set(DoubleSolenoid::Value::kReverse);
    } else if(!leftJoystick->GetTrigger()) {
        isHatchReady = true;
    	SmartDashboard::PutBoolean("Hatch State", isHatchReady);
        hatchDisconnect->Set(DoubleSolenoid::Value::kForward);
    }
    if(leftJoystick->GetRawButton(2)) {
        isHatchIntakeReady = false;
    	SmartDashboard::PutBoolean("Hatch Intake State", isHatchIntakeReady);
        hatchIntakeOutake->Set(DoubleSolenoid::Value::kReverse);
    } else if(!leftJoystick->GetRawButton(2)) {
        isHatchIntakeReady = true;
    	SmartDashboard::PutBoolean("Hatch Intake State", isHatchIntakeReady);
        hatchIntakeOutake->Set(DoubleSolenoid::Value::kForward);
    }

    //Control blocks for the elevator lifts.
    //Incorporates a purely manual and purely automatic control systens.
    //Angel remember to test the encoders.
    //Auto elevator uses PID closed loop and an array of values to coordinate which of seven unique heights to be at.
	if(abs(leftJoystick->GetY()) <= 0.05) {
		elevatorThrottle = 0;
		SmartDashboard::PutNumber("Elevator Throttle", elevatorThrottle);
	} else if(abs(leftJoystick->GetY()) > 0.05) {
		elevatorThrottle = leftJoystick->GetY();
		SmartDashboard::PutNumber("Elevator Throttle", elevatorThrottle);
	}
	elevatorMasterMotor->Set(ControlMode::PercentOutput, elevatorThrottle);


    //Control blocks for the actuator/joint/something.
    //Incorporates a purely manual and purely automatic control systens.
    //Test tho encoders.
    //Auto actuator uses PID closed loop and an array of values to coordinate which of three unique angle to be at. 
	if(leftJoystick->GetPOV() == 0) {
		actuatorThrottle = -1.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	} else if(leftJoystick->GetPOV() == 180) {
		actuatorThrottle = 1.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	} else {
		actuatorThrottle = 0.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	}
	actuatorMotor->Set(ControlMode::PercentOutput, actuatorThrottle);

    SmartDashboard::PutNumber("Master ELevator Power", elevatorMasterMotor->GetOutputCurrent());
    SmartDashboard::PutNumber("Slave Elevator Power", elevatorSlaveMotor->GetOutputCurrent());
}

void Robot::TeleopInit() {
    //Always init these to obvious-duh values.
    elevatorState = 0;
    isHatchReady = true;
    isElevatorOverride = false;
    isActuatorOverride = false;
    SHIFT_HIGH();
    hatchDisconnect->Set(DoubleSolenoid::Value::kReverse);
    hatchIntakeOutake->Set(DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
    //Drive control block.
    //Implements a simple threshold limit for both steering and throttle values.
    //Motor power assignment below block.
    //And remember that GetY() refers to vertical stick movement, GetX() refers to horizontal.
	if(abs(rightJoystick->GetY()) < 0.05) {
		robotThrottle = 0;
	} else {
		robotThrottle = rightJoystick->GetY();
	}
	if(abs(rightJoystick->GetX()) < 0.05) {
		robotSteer = 0;
	} else {
		robotSteer = (rightJoystick->GetX());
	}
    leftMasterMotor->Set(robotThrottle + (turnScaleDown*robotSteer));
	rightMasterMotor->Set(-(robotThrottle - (turnScaleDown*robotSteer)));
    leftSlaveMotor->Set(robotThrottle + (turnScaleDown*robotSteer));
    rightSlaveMotor->Set(-(robotThrottle - (turnScaleDown*robotSteer)));

    //Intake, ball and hatch, control block.
    if(leftJoystick->GetRawButton(5)) {
		hatchThrottle = 1.0;
	} else if(leftJoystick->GetRawButton(3)) {
		hatchThrottle = -1.0;
	} else {
		hatchThrottle = 0.0;
	}
	hatchRoller->Set(ControlMode::PercentOutput, hatchThrottle);
    
    //Transmission control block.
    //Implements a bool control system that flips a bool [mechanism]SoloTest instantly.
    //For all your single toggle needs!
    //Function-side code flips is[mechanism] and assigns is[mechanism]Ready bools for reuse and driver use.
    //Why am I writing literal paragraphs for these?
    if(rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE) && shiftingSoloTest) {
		if(isHighGear) {
			SHIFT_HIGH();
		} else if(!isHighGear) {
			SHIFT_LOW();
		}
		shiftingSoloTest = false;
	}
    //Once [mechanism]SoloTest flips in the block above, it awaits for the use to release the button.
    //Because [mechanism]SoloTest is checked in the first control loop and not in the second.
    //So, once the button is released shiftingSoloTest flips back ready for reuse.
	if(!rightJoystick->GetRawButton(rightJoystickMap::GEAR_CHANGE)) {
		shiftingSoloTest = true;
	}
    
    //Two simple control blocks to manage the hatch piston.
    //Persistent set to kForward to keep the piston in. 
    if(leftJoystick->GetTrigger()) {
        isHatchReady = false;
    	SmartDashboard::PutBoolean("Hatch State", isHatchReady);
        hatchDisconnect->Set(DoubleSolenoid::Value::kReverse);
    } else if(!leftJoystick->GetTrigger()) {
        isHatchReady = true;
    	SmartDashboard::PutBoolean("Hatch State", isHatchReady);
        hatchDisconnect->Set(DoubleSolenoid::Value::kForward);
    }
    if(leftJoystick->GetRawButton(2)) {
        isHatchIntakeReady = false;
    	SmartDashboard::PutBoolean("Hatch Intake State", isHatchIntakeReady);
        hatchIntakeOutake->Set(DoubleSolenoid::Value::kReverse);
    } else if(!leftJoystick->GetRawButton(2)) {
        isHatchIntakeReady = true;
    	SmartDashboard::PutBoolean("Hatch Intake State", isHatchIntakeReady);
        hatchIntakeOutake->Set(DoubleSolenoid::Value::kForward);
    }

    //Control blocks for the elevator lifts.
    //Incorporates a purely manual and purely automatic control systens.
    //Angel remember to test the encoders.
    //Auto elevator uses PID closed loop and an array of values to coordinate which of seven unique heights to be at.
	if(abs(leftJoystick->GetY()) <= 0.05) {
		elevatorThrottle = 0;
		SmartDashboard::PutNumber("Elevator Throttle", elevatorThrottle);
	} else if(abs(leftJoystick->GetY()) > 0.05) {
		elevatorThrottle = leftJoystick->GetY();
		SmartDashboard::PutNumber("Elevator Throttle", elevatorThrottle);
	}
	elevatorMasterMotor->Set(ControlMode::PercentOutput, elevatorThrottle);


    //Control blocks for the actuator/joint/something.
    //Incorporates a purely manual and purely automatic control systens.
    //Test tho encoders.
    //Auto actuator uses PID closed loop and an array of values to coordinate which of three unique angle to be at. 
	if(leftJoystick->GetPOV() == 0) {
		actuatorThrottle = -1.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	} else if(leftJoystick->GetPOV() == 180) {
		actuatorThrottle = 1.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	} else {
		actuatorThrottle = 0.0;
		SmartDashboard::PutNumber("Actuator Throttle", actuatorThrottle);
	}
	actuatorMotor->Set(ControlMode::PercentOutput, actuatorThrottle);

    if() {
        leftMasterMotor->RestoreFactoryDefaults();
    }

    SmartDashboard::PutNumber("Master ELevator Power", elevatorMasterMotor->GetOutputCurrent());
    SmartDashboard::PutNumber("Slave Elevator Power", elevatorSlaveMotor->GetOutputCurrent());
}

void Robot::TestPeriodic() {
    //Do not put anything here.
    //You have been warned.
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif