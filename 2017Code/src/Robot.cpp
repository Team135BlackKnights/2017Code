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
		autonomousChooser.AddDefault("Middle Gear No BaseLine", new AutonomousCommand(AutonomousSelection::MiddleGear, BaseLinePath::NoBaseLine));
		autonomousChooser.AddObject("Middle Gear BaseLine to the Right", new AutonomousCommand(AutonomousSelection::MiddleGear, BaseLinePath::MiddleGearRight));
		autonomousChooser.AddObject("Middle Gear BaseLine to the Left", new AutonomousCommand(AutonomousSelection::MiddleGear, BaseLinePath::MiddleGearLeft));
		autonomousChooser.AddObject("Right Gear with No BaseLine", new AutonomousCommand(AutonomousSelection::RightGear, BaseLinePath::NoBaseLine));
		autonomousChooser.AddObject("Right Gear with BaseLine", new AutonomousCommand(AutonomousSelection::RightGear, BaseLinePath::SideGear));
		autonomousChooser.AddObject("Left Gear with No BaseLine", new AutonomousCommand(AutonomousSelection::LeftGear, BaseLinePath::NoBaseLine));
		autonomousChooser.AddObject("Left Gear with BaseLine", new AutonomousCommand(AutonomousSelection::LeftGear, BaseLinePath::SideGear));
		frc::SmartDashboard::PutData("Auto Modes", &autonomousChooser);

		CommandBase::agitator->InitializeAgitatorMotor(COMPETITION_BOT);
		CommandBase::collection->InitializeCollectionMotor(COMPETITION_BOT);
		CommandBase::driveTrain->InitializeDriveTrainMotors(COMPETITION_BOT);
		CommandBase::driveTrain->ConfigureDriveTrainEncoders(COMPETITION_BOT);
		CommandBase::gearHolder->InitializeGearHolderMotor(COMPETITION_BOT);
		CommandBase::lidars->InitializeLidars();
		CommandBase::liftHang->InitializeLiftHang(COMPETITION_BOT);
		CommandBase::shooter->InitializeShooterMotor(COMPETITION_BOT);
		CommandBase::shooter->ConfigureShooterMotorEncoder();
		CommandBase::shooterHood->InitializeShooterHoodMotor(COMPETITION_BOT);
		CommandBase::shooterHood->ConfigureShooterHoodEncoder();
		CommandBase::ultrasonicSensor->InitializeUltrasonicSensor();

		CameraServer::GetInstance()->StartAutomaticCapture();
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
			//autonomousCommand.reset(new ExampleCommand());
		} */

		autonomousCommand.reset(autonomousChooser.GetSelected());

		if (autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}
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
