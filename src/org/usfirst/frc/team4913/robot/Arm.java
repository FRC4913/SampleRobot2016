/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.usfirst.frc.team4913.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author michellephan
 */
public class Arm {
    int speedControl;
    
    double k = .005;
    
    int upperLimit = 1000;
    int startPID = 800;
    
    Talon armControl;
    Encoder enc;
    
    Joystick joy;
    
    public Arm(int DIO1,int DIO2, int speedControl) {
        
        joy = new Joystick(1);
        
        armControl = new Talon(speedControl);
        enc = new Encoder(DIO1, DIO2);
        
        enc.setDistancePerPulse(1);
        enc.reset();
        
        
    }
    
    public void run() {
        if (joy.getRawButton(1) && enc.getDistance()<upperLimit) {
            armControl.set(1);
        }
        else if (enc.getDistance()>=upperLimit) {
            armControl.set(0);
        }
        if (joy.getRawButton(2) && enc.getDistance()>0) {
            armControl.set(-1);
        }
        else if (enc.getDistance()<=-upperLimit){
            armControl.set(0);
        }
        
        if (!joy.getRawButton(1) && !joy.getRawButton(2)){
            armControl.set(0);
        }
        print();
    }
    
    public void runPID(){
        if (joy.getRawButton(1) && enc.getDistance()<upperLimit) {
            if (enc.getDistance()> startPID) {
                double speed = (upperLimit-enc.getDistance())* k;
                armControl.set(speed);
            }
            else 
                armControl.set(1);
        }
        
        else if (enc.getDistance() >= upperLimit) {
            armControl.set(0);
        }
        if (joy.getRawButton(2) && enc.getDistance()>=0){
            if (enc.getDistance() < -startPID) {
                double speed = (-upperLimit + enc.getDistance())*k;
                armControl.set(speed);
            }
            else
                armControl.set(-1);
        }
        
        if (!joy.getRawButton(1) && !joy.getRawButton(2)) {
            armControl.set(0);
        }
        
    }
    
    public void print() {
        SmartDashboard.putNumber("encoder: ", enc.getDistance());
        SmartDashboard.putBoolean("direction: ", enc.getDirection());
        SmartDashboard.putNumber("motor value: ", armControl.get());
        
    }
}
