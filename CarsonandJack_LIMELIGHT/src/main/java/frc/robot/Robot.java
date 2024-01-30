// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.LimelightCMD;
//import frc.robot.commands.LimelightCMD;
import frc.robot.subsystems.LimelightSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command getLimelightValuesCommand;


  private RobotContainer m_robotContainer;


    //AHRS ahrs;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //ahrs = new AHRS(SPI.Port.kMXP);

    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();



    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    SmartDashboard.setDefaultNumber("Auto Path", 0);

    //SmartDashboard.putNumber(   "getPitch",            ahrs.getPitch());
    //SmartDashboard.putNumber(   "getYaw",            ahrs.getYaw());
    //SmartDashboard.putNumber(   "getRoll",            ahrs.getRoll());

    // Roll (forward/backward) and Pitch (Side to Side) for docking station

    SmartDashboard.putNumber("Current Arm2 Motor Pos",m_robotContainer.theArmSystem.CurrentArm2MotorPos() );
    SmartDashboard.putNumber("Current Arm3 Motor Pos",m_robotContainer.theArmSystem.CurrentArm3MotorPos() );
    SmartDashboard.putNumber("Current Arm2 Alt Pos",m_robotContainer.theArmSystem.CurrentArm2AltPos() );
    SmartDashboard.putNumber("Current Arm3 Alt Pos",m_robotContainer.theArmSystem.CurrentArm3AltPos() );
    SmartDashboard.putNumber("Arm2 Applied Out",m_robotContainer.theArmSystem.CurrentArm2MotorAppliedOutput() );
    SmartDashboard.putNumber("Arm2F Applied Out",m_robotContainer.theArmSystem.CurrentArm2FMotorAppliedOutput() );
    SmartDashboard.putNumber("Arm2 Velocity",m_robotContainer.theArmSystem.Arm2Velocity() );

    SmartDashboard.putNumber("Arm3 Applied Out",m_robotContainer.theArmSystem.Arm3AppliedOutput() );
    SmartDashboard.putNumber("Arm3 Velocity",m_robotContainer.theArmSystem.Arm3Velocity() );

    SmartDashboard.putNumber("Intake Position",m_robotContainer.theIntakeSubsystem.CurrentIntakePosition());

    //The In Button
    if(m_robotContainer.theArmSystem.CurrentArm3AltPos() < .18){
        //Cone IN 
        IntakeConstants.IN = .75;
    }else{
        if(m_robotContainer.theArmSystem.CurrentArm2AltPos() > .1){
          //Cone IN while Arm 2 is up high
          IntakeConstants.IN = .5;
        }else{
          //Cube IN
          IntakeConstants.IN = -.75;
        }
    }
    
    //The out button
    if(m_robotContainer.theArmSystem.CurrentArm3AltPos() < .1){
        //Cube OUT
        IntakeConstants.OUT = .75;
    }else if(m_robotContainer.theArmSystem.CurrentArm3AltPos() > .35){
        //Cone OUT
        IntakeConstants. OUT = -.75;
    }else{ //check arm 2 because we can't tell based on arm 3
        if( m_robotContainer.theArmSystem.CurrentArm2AltPos() > .15){
            //Cube OUT
            IntakeConstants.OUT = .75;
        }else {
            //Cone OUT
            IntakeConstants.OUT = -.6;
        }
    }

    SmartDashboard.putNumber("IN",IntakeConstants.IN);
    SmartDashboard.putNumber("OUT",IntakeConstants.OUT);


  }

  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    AutoConstants.AutoRoute = SmartDashboard.getNumber("Auto Path",1);
    AutoConstants.AutoDistance = SmartDashboard.getNumber("Auto Distance(Testing)",0);


    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //Limelight commands for auton
    getLimelightValuesCommand = m_robotContainer.getLimelightValuesCommand();
    getLimelightValuesCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //Limelight commands for auton
    getLimelightValuesCommand = m_robotContainer.getLimelightValuesCommand();
    getLimelightValuesCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
