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

	private double motorMinSpeed = 0.1;
	private int encUpperLimit = 1000;
	private int encLowerLimit = 0;
	private int startPIDDown = 200;
	private int startPIDUp = 800;
	private double k = .005; // proportionality constant for PID

	private Talon armMotor;
	private Encoder enc;


	public Arm() {
		armMotor = new Talon(ARM_CHANNEL);
		enc = new Encoder(ENC_SOURCE_1, ENC_SOURCE_2);
		enc.setDistancePerPulse(DISTANCE_PER_PULSE);
		enc.reset();
	}

	public void armUp(boolean pidControl) {
		double distance = enc.getDistance();
		if (distance > encLowerLimit) {
			if (pidControl && distance < startPIDDown) {
				double speed = distance * k;
				speed = speed > motorMinSpeed ? speed : motorMinSpeed;
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
		if (distance < encUpperLimit) {
			if (pidControl && distance > startPIDUp) {
				double speed = (encUpperLimit - distance) * k;
				speed = speed > motorMinSpeed ? speed : motorMinSpeed;
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

	public int getEncUpperLimit() {
		return encUpperLimit;
	}

	public void setEncUpperLimit(int encUpperLimit) {
		this.encUpperLimit = encUpperLimit;
	}

	public double getK() {
		return k;
	}

	public void setK(double k) {
		this.k = k;
	}

	public double getMotorMinSpeed() {
		return motorMinSpeed;
	}

	public void setMotorMinSpeed(double motorMinSpeed) {
		this.motorMinSpeed = motorMinSpeed;
	}

}
