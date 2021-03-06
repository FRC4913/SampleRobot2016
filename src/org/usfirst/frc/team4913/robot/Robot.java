package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	CANTalon frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;

	private static final int FRONT_LEFT = 4;
	private static final int REAR_LEFT = 3;
	private static final int FRONT_RIGHT = 1;
	private static final int REAR_RIGHT = 2;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		stick = new Joystick(0);
		frontLeftMotor = new CANTalon(FRONT_LEFT);
		rearLeftMotor = new CANTalon(REAR_LEFT);
		frontRightMotor = new CANTalon(FRONT_RIGHT);
		rearRightMotor = new CANTalon(REAR_RIGHT);

		rearLeftMotor.changeControlMode(TalonControlMode.Follower);
		rearRightMotor.changeControlMode(TalonControlMode.Follower);
		rearLeftMotor.set(FRONT_LEFT);
		rearRightMotor.set(FRONT_RIGHT);

		myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		if (autoLoopCounter < 100) // Check if we've completed 100 loops
									// (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); // drive forwards half speed
			autoLoopCounter++;
		} else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		myRobot.arcadeDrive(stick);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

}
