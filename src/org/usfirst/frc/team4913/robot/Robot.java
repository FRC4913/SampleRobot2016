package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
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
	CameraServer server;

	private static final int STRAFE_SECONDS = 3;

	private static final int FRONT_LEFT = 4;
	private static final int REAR_LEFT = 3;
	private static final int FRONT_RIGHT = 1;
	private static final int REAR_RIGHT = 2;
	private static final int CAMERA_QUALITY = 50; // can be set to 0 - 100

	private static final boolean PID_ENABLED = true;
	Arm arm;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		stick = new Joystick(0);
		arm = new Arm();
		frontLeftMotor = new CANTalon(FRONT_LEFT);
		rearLeftMotor = new CANTalon(REAR_LEFT);
		frontRightMotor = new CANTalon(FRONT_RIGHT);
		rearRightMotor = new CANTalon(REAR_RIGHT);

		rearLeftMotor.changeControlMode(TalonControlMode.Follower);
		rearRightMotor.changeControlMode(TalonControlMode.Follower);
		rearLeftMotor.set(FRONT_LEFT);
		rearRightMotor.set(FRONT_RIGHT);

		server = CameraServer.getInstance();
		server.setQuality(CAMERA_QUALITY);
		server.startAutomaticCapture("cam0");

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
		if (stick.getRawButton(1)) {
			arm.armUp(PID_ENABLED);
		} else if (stick.getRawButton(0)) {
			arm.armDown(PID_ENABLED);
		} else {
			arm.armStop();
		}

		boolean strafeRight = false;
		if (stick.getRawButton(4)) {
			strafeRight = !strafeRight;

			for (int i = 0; i < STRAFE_SECONDS; i++) {
				myRobot.drive(.5, .5); // rotate right for constant sec
			}
			while (strafeRight) {
				myRobot.drive(.5, 0); // drive forward
				if (stick.getRawButton(4)) {
					strafeRight = !strafeRight;
				}
			}

			for (int i = 0; i < STRAFE_SECONDS; i++) {
				myRobot.drive(.5, -.5);
			} // rotate left for constant sec

			myRobot.drive(0, 0);

		}
		if (stick.getRawButton(5)) {
			myRobot.drive(.5, -.5);
			Timer.delay(STRAFE_SECONDS); // rotate left for constant sec
			while (stick.getRawButton(4)) {
				myRobot.drive(.5, 0); // drive forward

			}
			myRobot.drive(.5, .5);
			Timer.delay(STRAFE_SECONDS);// rotate right for constant sec

		}
		myRobot.arcadeDrive(stick);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

}
