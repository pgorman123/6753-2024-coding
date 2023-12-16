// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmSetCMD;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


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
    timer2 = 0;


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
  public void teleopPeriodic() {
    //this sees a cube and picks it up
    if (ColorSensor = 2/*looking for color purple */){
     timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new CubePickup(true)); 
        while (timer<1000) 
          (new IntakeRunWheelsCMD(theIntakeSubsystem, -.75)).
      list.of(new IntakeRunWheelsCMD(theIntakeSubsystem, 0),(new ArmReset(true)));
    }
    //this if then statement just sees a cone and picks it up
    if (new JoystickButton(m_driverController2,OperatorConstants.ConePickup).whileTrue(new ConePickup()));{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));
      }if (ColorSensor = 3 /*looking for yellow */);{
      exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0)),(new ArmReset(true)));
    }
    //this makes the cube high placing automatic
    if ( new JoystickButton(m_driverController2, OperatorConstants.CubePlaceHigh).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)),
       new Translation2d(0, 0.1524, new Rotation2d(.5)), new CubePlaceHigh());
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)),
       new Translation2d(0, 0.1524, new Rotation2d(0)));
    }
    //Cube mid
    if ( new JoystickButton(m_driverController2, OperatorConstants.CubePlaceMid).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)),
       new Translation2d(0, 0.1524, new Rotation2d(.5)), new CubePlaceMid(true));
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)),
       new Translation2d(0, 0.1524, new Rotation2d(0)));
    }

     //Cube low
    if ( new JoystickButton(m_driverController2, OperatorConstants.CubePlaceLow).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)),
       new Translation2d(0, 0.1524, new Rotation2d(.5)), new CubePlaceLow(true));
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)),
       new Translation2d(0, 0.1524, new Rotation2d(0)));
    }

    //this makes the cone high placing automatic
    if ( new JoystickButton(m_driverController2, OperatorConstants.ConePlaceHigh).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)), 
      new Translation2d(0, 0.1524, new Rotation2d(.5)), (new ConePlaceHigh (true)));
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)), 
      new Translation2d(0, 0.1524, new Rotation2d(0)));
    }
     //this makes the cone high placing automatic
    if ( new JoystickButton(m_driverController2, OperatorConstants.ConePlaceMid).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)), 
      new Translation2d(0, 0.1524, new Rotation2d(.5)), (new ConePlaceMid (true)));
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)), 
      new Translation2d(0, 0.1524, new Rotation2d(0)));
    }

     //this makes the cone Low placing automatic
    if ( new JoystickButton(m_driverController2, OperatorConstants.ConePlaceLow).whileTrue());{
      timer = 0; exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0.5)), 
      new Translation2d(0, 0.1524, new Rotation2d(.5)), (new ConePlaceLow (true)));
         while (t<1000)
          List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));
      List.of (new IntakeRunWheelsCMD(theIntakeSubsystem, 0), (new ArmReset(true)), 
      new Translation2d(0, 0.1524, new Rotation2d(0)));
    }
    
    //endgame autoballance
    if (timer2>135000)
      if(driveSubsystem.getRoll() < -10) {
      //drive forward
       driveSubsystem.drive(.07,0,0,false);
      SmartDashboard.putString("AutoBalance", "forward");
      } else if(driveSubsystem.getRoll() > 10) {
      //drive backwards
      driveSubsystem.drive(-.07,0,0,false);
      SmartDashboard.putString("AutoBalance", "backward");
      } else {
      //stop
      driveSubsystem.setX();
      SmartDashboard.putString("AutoBalance", "stop");
      }                   
      
      if (new JoystickButton(theJoystick,5).whileTrue());{
        if(driveSubsystem.getRoll() < -10) {
          //drive forward5
           driveSubsystem.drive(.07,0,0,false);
          SmartDashboard.putString("AutoBalance", "forward");
       } else if(driveSubsystem.getRoll() > 10) {
          //drive backwards
          driveSubsystem.drive(-.07,0,0,false);
          SmartDashboard.putString("AutoBalance", "backward");
       } else {
          //stop
          driveSubsystem.setX();
          SmartDashboard.putString("AutoBalance", "stop");
       }
      }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
