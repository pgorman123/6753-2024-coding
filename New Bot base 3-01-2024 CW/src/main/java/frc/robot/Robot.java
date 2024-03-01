// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
//import frc.robot.subsystems.LimelightHelpers; this was commented out to build code
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.setDefaultNumber("Auto Path", 0);
    //LimelightHelpers.LimelightResults.getBotpose();
    //double[] botpose = LimelightHelpers.getBotPose("");this was commented out to build code
    //double tx = LimelightHelpers.getTX("");this was commented out to build code
    //double ty = LimelightHelpers.getTY("");this was commented out to build code

    //Array Jeff = 


    
   //LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults(""); this was commented out to build code
   //LimelightHelpers.Results
    //double[] botposeRed = llresults.results.botpose_wpired;
    //double pipelineLatency = llresults.results.latency_pipeline;
    //LimelightHelpers.LimelightTarget_Fiducial = llresults.results.targets_Fiducials;
   
  
  
  
  // public static double [] getBotpose("") {
      //return getLimelightNTDoubleArray(limelightName, "botpose");
    
  // }
    //SmartDashboard.putNumber(   "getPitch",            ahrs.getPitch());
    //SmartDashboard.putNumber(   "getYaw",            ahrs.getYaw());
    //SmartDashboard.putNumber(   "getRoll",            ahrs.getRoll());

    //SmartDashboard.putNumber("Current ShooterArm Motor Pos",m_robotContainer.theShooterArmSystem.CurrentShooterArmMotorPos() );
    SmartDashboard.putNumber("Current IntakeArm Motor Pos",m_robotContainer.theNewIntakeArmSubsystem.CurrentIntakeArmMotorPos() );
   // SmartDashboard.putNumber("Current ShooterArm Alt Pos",m_robotContainer.theShooterArmSystem.CurrentShooterArmAltPos() );
    SmartDashboard.putNumber("Current IntakeArm Alt Pos",m_robotContainer.theNewIntakeArmSubsystem.CurrentIntakeArmAltPos() );
    //SmartDashboard.putNumber("ShooterArm Applied Out",m_robotContainer.theShooterArmSystem.CurrentShooterArmMotorAppliedOutput() );
    //SmartDashboard.putNumber("ShooterArmF Applied Out",m_robotContainer.theArmSystem.CurrentShooterArmFMotorAppliedOutput() );
    //SmartDashboard.putNumber("ShooterArm Velocity",m_robotContainer.theShooterArmSystem.ShooterArmVelocity() );

    SmartDashboard.putNumber("IntakeArm Applied Out",m_robotContainer.theNewIntakeArmSubsystem.IntakeArmAppliedOutput() );
    SmartDashboard.putNumber("IntakeArm Velocity",m_robotContainer.theNewIntakeArmSubsystem.IntakeArmVelocity() );

    SmartDashboard.putNumber("Intake Position",m_robotContainer.theIntakeSubsystem.CurrentIntakePosition());
    //SmartDashboard.putNumber("Botpose",botpose[2]); this was commented out to build code
    //SmartDashboard.putNumber("tx", tx); this was commented out to build code
    //SmartDashboard.putNumber("ty", ty); this was commented out to build code


    IntakeConstants.IN = 0.3;
    IntakeConstants.OUT = -0.3;

    
    ShooterConstants.IN = 1;
    ShooterConstants.OUT = -1;
    

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
