package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap;
import frc.robot.Util.Subsystems;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
public class DriveSubsystem extends PIDSubsystem {
    public TalonSRX left1 = new TalonSRX(RobotMap.left1);
    public TalonSRX left2 = new TalonSRX(RobotMap.left2);
    public TalonSRX right1 = new TalonSRX(RobotMap.right1);
    public TalonSRX right2 = new TalonSRX(RobotMap.right2);
    public AHRS ahrs;
    public DriveSubsystem () {
        super("Drive", 0, 0, 0);
    }
    public void tankDrive(double left, double right) {
        left1.set(ControlMode.PercentOutput, left);
        left2.set(ControlMode.PercentOutput, left);
        right1.set(ControlMode.PercentOutput, -right);
        right2.set(ControlMode.PercentOutput, -right);
    }
    public void arcadeDrive(double speed, double turn) {
        tankDrive (speed-turn, speed+turn);
    }
    protected void initDefaultCommand() {
    	left2.set(ControlMode.Follower, 0);
    	right2.set(ControlMode.Follower, 2);
    }
    public double returnPIDInput() {
        return ahrs.pidGet();
    }
    
    public void usePIDOutput (double output) {
        
    }
    public void PIDSubsystem(){

    }
}