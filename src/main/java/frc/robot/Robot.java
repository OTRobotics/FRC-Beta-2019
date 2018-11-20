package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.Util.Subsystems;

public class Robot extends IterativeRobot {

  public static OI oi;

  Command DriveCommands;
  

  Subsystems sub;
  Command autonomousCommand;
  
  SendableChooser<String> chooser;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  public void robotInit() {
      sub = new Subsystems();
      //oi = new OI();
      DriveCommands = new DriveCommand();
      
      
      chooser = new SendableChooser<String>();
     
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  public void disabledInit(){

  }

  public void disabledPeriodic() {
      Scheduler.getInstance().run();
  }

  public void autonomousInit() {
      if (autonomousCommand != null) autonomousCommand.start();
  }

  /**
   * This function is called periodically during autonomous
   */
  public void autonomousPeriodic() {
      Scheduler.getInstance().run();
  }

  public void teleopInit() {
      DriveCommands.start();
     
      // This makes sure that the autonomous stops running when
      // teleop starts running. If you want the autonomous to
      // continue until interrupted by another command, remove
      // this line or comment it out.
      if (autonomousCommand != null) autonomousCommand.cancel();
  }

  /**
   * This function is called periodically during operator control
   */
  public void teleopPeriodic() {
      Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode
   */
  public void testPeriodic() {
      LiveWindow.run();
  }
}