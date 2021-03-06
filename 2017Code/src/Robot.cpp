#include <memory>

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "CommandBase.h"

#include "Commands/AutonomousCommand.h"

#include "RobotMap.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() override {
		/*autonomousChooser.AddObject("Middle Gear", new AutonomousCommand(AutonomousSelection::MiddleGear, SecondTask::None));
		autonomousChooser.AddObject("Middle Gear Shoot Right", new AutonomousCommand(AutonomousSelection::MiddleGear, SecondTask::MiddleGearShootRight));
		autonomousChooser.AddDefault("Middle Gear Shoot Left", new AutonomousCommand(AutonomousSelection::MiddleGear, SecondTask::MiddleGearShootLeft));
		autonomousChooser.AddObject("Right Gear", new AutonomousCommand(AutonomousSelection::RightGear, SecondTask::None));
		autonomousChooser.AddObject("Left Gear", new AutonomousCommand(AutonomousSelection::LeftGear, SecondTask::None));
		autonomousChooser.AddObject("Right Gear and Shoot", new AutonomousCommand(AutonomousSelection::RightGear, SecondTask::SideGearShoot));
		autonomousChooser.AddObject("Left Gear and Shoot", new AutonomousCommand(AutonomousSelection::LeftGear, SecondTask::SideGearShoot));
		autonomousChooser.AddObject("Close Shot Right With BaseLine", new AutonomousCommand(AutonomousSelection::CloseShotShooterRight, SecondTask::CloseShotBaseLine));
		autonomousChooser.AddObject("Close Shot Left With BaseLine", new AutonomousCommand(AutonomousSelection::CloseShotShooterLeft, SecondTask::CloseShotBaseLine));
		autonomousChooser.AddObject("BaseLine Only", new AutonomousCommand(AutonomousSelection::BaseLine, SecondTask::None));
		autonomousChooser.AddObject("Right 40Kpa", new AutonomousCommand(AutonomousSelection::RightKPaAutonomous, SecondTask::None));
		autonomousChooser.AddObject("Left 40KPa", new AutonomousCommand(AutonomousSelection::LeftKPaAutonomous, SecondTask::None)); */

		CommandBase::agitator->InitializeAgitatorMotor(COMPETITION_BOT);
		//CommandBase::cameraServo->InitializeCameraServo();
		CommandBase::collection->InitializeCollectionMotor(COMPETITION_BOT);
		CommandBase::driveTrain->InitializeDriveTrainMotors(COMPETITION_BOT);
		CommandBase::driveTrain->ConfigureDriveTrainEncoders(COMPETITION_BOT);
		CommandBase::driveTrain->InitializeDriveTrainPID();
		CommandBase::driveTrain->InitializeDriveStraightWithGyro(COMPETITION_BOT);
		CommandBase::gearHolder->InitializeGearHolderMotor(COMPETITION_BOT);
		CommandBase::lidars->InitializeLidarsAndI2CMultiplexer();
		CommandBase::lidars->InitializeLidarPowerEnablePin();
		CommandBase::liftHang->InitializeLiftHang(COMPETITION_BOT);
		CommandBase::liftHang->InitializeLiftHangEncoder();
		CommandBase::pdp->InitializePDP();
		CommandBase::shooter->InitializeShooterMotor(COMPETITION_BOT);
		CommandBase::shooter->ConfigureShooterMotorEncoder(COMPETITION_BOT);
		CommandBase::shooterHood->InitializeShooterHoodMotor(COMPETITION_BOT);
		CommandBase::shooterHood->ConfigureShooterHoodEncoder();
		CommandBase::ultrasonicSensor->InitializeUltrasonicSensors();

		autonomousChooser.AddDefault("Autonomous", new AutonomousCommand());
		frc::SmartDashboard::PutData("Autonomouss Modes", &autonomousChooser);

		//CameraServer::GetInstance()->StartAutomaticCapture();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {

	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		/*std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			//autonomousCommand.reset(new MyAutoCommand());
		}
		else {
			//autonomousCommand.reset(n-ew ExampleCommand());
		} */

		CommandBase::liftHang->ZeroLiftHangEncoder();
		//autonomousCommand.reset(autonomousChooser.GetSelected());

		autonomousCommand.reset(autonomousChooser.GetSelected());
		autonomousCommand->Start();
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
		CommandBase::driveTrain->is_aiming = false;
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> autonomousChooser;
};

START_ROBOT_CLASS(Robot)
