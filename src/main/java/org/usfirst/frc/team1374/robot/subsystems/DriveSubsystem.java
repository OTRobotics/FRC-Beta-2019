package org.usfirst.frc.team1374.robot.subsystems;

import org.usfirst.frc.team1374.robot.RobotMap;
import org.usfirst.frc.team1374.robot.Util.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DriveSubsystem extends PIDSubsystem {

    public static int speedconst = 10;
    public static double angle = 0;
    
    public TalonSRX left1 = new TalonSRX(RobotMap.left1);
    public TalonSRX left2 = new TalonSRX(RobotMap.left2);
    public TalonSRX right1 = new TalonSRX(RobotMap.right1);
    public TalonSRX right2 = new TalonSRX(RobotMap.right2);

    public AHRS ahrs;

    public Compressor c = new Compressor(RobotMap.compressor);
    public static DoubleSolenoid shift = new DoubleSolenoid(RobotMap.shift1, RobotMap.shift2);

    public DriveSubsystem () {
        super("Drive", 0, 0, 0);
    }

    public void CompressorControl (){
    	c.setClosedLoopControl(true);    
    }
    
    public void setPIDDRIVE () {
    	left1.set(ControlMode.Position, 0);
    	right1.set(ControlMode.Position, 0);
    	left2.set(ControlMode.Follower, 0);
    	right2.set(ControlMode.Follower, 2);
    }
    
    public void setREGULARDRIVE (){
    	left1.set(ControlMode.Velocity, 0);
    	right1.set(ControlMode.Velocity, 0);
    	left2.set(ControlMode.Follower, 0);
    	right2.set(ControlMode.Follower, 2);
    }
    
    public void distanceDrive (int distance) {
    	left1.set(ControlMode.Position, distance * 4096);
    	left2.set(ControlMode.Follower, 0);
    	right1.set(ControlMode.Position, distance * 4096);
    	right2.set(ControlMode.Follower, 2);
    }
    
    protected void initDefaultCommand () {
    	left2.set(ControlMode.Follower, 0);
    	right2.set(ControlMode.Follower, 2);
    }
    
    public void tankDrive (double left, double right) {
        left1.set(ControlMode.PercentOutput, left);
        left2.set(ControlMode.PercentOutput, left);
        right1.set(ControlMode.PercentOutput, -right);
        right2.set(ControlMode.PercentOutput, -right);
    }

    public void distanceTankDrive (double left, double right) {
    	left1.set(ControlMode.Position, 500);
		left2.set(ControlMode.Follower, 0);
		right1.set(ControlMode.Position, 500);
		right2.set(ControlMode.Follower, 2);
    }
    
    public void arcadeDrive(double speed, double turn) {
        tankDrive (speed-turn, speed+turn);
    }
    
    public String distanceArcadeDrive (double speed, double turn) {
    	tankDrive ((speed-turn) * 4096 * 500.0 / 600, (speed+turn) * 4096 * 500.0 / 600);
    	return (speed-turn) * 4096 * 500.0 / 600 + "";
    }
   
    public void shiftGear (boolean up, boolean down) {
        //msg to future chris up and down is in drive commands and becomes 
        //true/false when pressed and just change it from other things when changing
    	if (up) {
    		shift.set(Value.kForward);
    	}
    	
    	else if (down) {
    		shift.set(Value.kReverse);
    	}
    	
    }

    public void resetGyroAngle () {
        ahrs.reset();
        angle = 0;
    }

    public double returnPIDInput () {
        return ahrs.pidGet();
    }
    
    public void usePIDOutput (double output) {
        
    }
    
}