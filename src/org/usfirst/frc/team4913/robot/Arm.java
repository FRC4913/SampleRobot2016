/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author michellephan
 */
public class Arm {
	private static final int ARM_CHANNEL = 5;
	private static final int ENC_SOURCE_1 = 0;
	private static final int ENC_SOURCE_2 = 1;
	private static final int DISTANCE_PER_PULSE = 1;

	private static final int ENC_UPPER_LIMIT = 1000;
	private static final int ENC_LOWER_LIMIT = 0;
	private static final int START_PID_DOWN = 200;
	private static final int START_PID_UP = 800;

	private Talon armMotor;
	private Encoder enc;

	private double k = .005; // proportionality constant for PID

	public Arm() {
		armMotor = new Talon(ARM_CHANNEL);
		enc = new Encoder(ENC_SOURCE_1, ENC_SOURCE_2);
		enc.setDistancePerPulse(DISTANCE_PER_PULSE);
		enc.reset();
	}

	public void armUp(boolean pidControl) {
		double distance = enc.getDistance();
		if (distance > ENC_LOWER_LIMIT) {
			if (pidControl && distance < START_PID_DOWN) {
				double speed = distance * k;
				armMotor.set(-speed);
			} else
				armMotor.set(-1);
		}
		else
			armMotor.set(0);
		print();
	}

	public void armDown(boolean pidControl) {
		double distance = enc.getDistance();
		if (distance < ENC_UPPER_LIMIT) {
			if (pidControl && distance > START_PID_UP) {
				double speed = (ENC_UPPER_LIMIT - distance) * k;
				armMotor.set(speed);
			} else
				armMotor.set(1);
		} else {
			armMotor.set(0);
		}
		print();
	}

	public void armStop(){
		armMotor.set(0);
	}

	private void print() {
		SmartDashboard.putNumber("encoder: ", enc.getDistance());
		SmartDashboard.putBoolean("direction: ", enc.getDirection());
		SmartDashboard.putNumber("motor value: ", armMotor.get());

	}
}
