// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmSetCMD;
import frc.robot.commands.IntakeRunWheelsCMD;
//import frc.robot.commands.IntakeRunWheelsToPosition;
// import frc.robot.commands.IntakeToggleCMD;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.IntakeSetCMD;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // The driver's controller  XboxController m_driverController1 = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_driverController1 = new XboxController(OperatorConstants.kDriverController1Port);
  XboxController m_driverController2 = new XboxController(OperatorConstants.kDriverController2Port);

  //Define an arm subsystem
  public final ArmSubsystem theArmSystem = new ArmSubsystem();

  //Define an intake subsystem
  private final IntakeSubsystem theIntakeSubsystem = new IntakeSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Arm positions
        //Return to Zero 
    new JoystickButton(m_driverController2,OperatorConstants.ArmReset).whileTrue(new ArmSetCMD(theArmSystem, 0,0, 0));
        //Pick up Cube and Place Ground 
    new JoystickButton(m_driverController2,OperatorConstants.CubePickup).whileTrue(new ArmSetCMD(theArmSystem, 0,15, 60));
        //Low Cube Place
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceLow).whileTrue(new ArmSetCMD(theArmSystem, 0, 0, 0 ));
        //Medium Cube Place 
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceMid).whileTrue(new ArmSetCMD(theArmSystem, 0, 43, 65));
        //High Cube Place 
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceHigh).whileTrue(new ArmSetCMD(theArmSystem, 0, 55, 70));
         //High Cone place
    new JoystickButton(m_driverController2,OperatorConstants.ConePlaceHigh).whileTrue(new ArmSetCMD(theArmSystem, 0,80, 132));
        //Medium Cone Place 
    new JoystickButton(m_driverController2, OperatorConstants.ConePlaceMid).whileTrue(new ArmSetCMD(theArmSystem, 0, 65, 132));
        //Ground Cone Place 
    new JoystickButton(m_driverController2, OperatorConstants.ConePlaceLow).whileTrue(new ArmSetCMD(theArmSystem, 0, 0, 35));
    
        //Intake Out Cube
    new JoystickButton(m_driverController1,OperatorConstants.CubeOutake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));
        //Intake In Cube
    new JoystickButton(m_driverController1,OperatorConstants.CubeIntake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
        //Intake In Cone
    new JoystickButton(m_driverController1,OperatorConstants.ConeIntake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));
        //Intake Out Cone
    new JoystickButton(m_driverController1,OperatorConstants.ConeOutake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
        //human player pick up
    new JoystickButton(m_driverController2,OperatorConstants.ConePickup).whileTrue(new ArmSetCMD(theArmSystem, 0,0, 10));
      
    
    
    //Toggle
   // new JoystickButton(m_driverController2,1).whileTrue(new IntakeToggleCMD(theIntakeSubsystem));
        //human player pick up
    //new JoystickButton(m_driverController2,7).whileTrue(new ArmSetCMD(theArmSystem, 0,0, 0));
      

    //new JoystickButton(m_driverController, 11).onTrue(new IncrementArmCMD(theArmSystem, 1));
    //new JoystickButton(m_driverController, 12).onTrue(new IncrementArmCMD(theArmSystem, -1));

    //m_driverController.a().whileTrue(new testcmd());
    // Configure default commands


    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController1.getLeftY() * Math.abs(m_driverController1.getLeftY()) * DriveConstants.DriverInputRedution, 0.06),
                MathUtil.applyDeadband(-m_driverController1.getLeftX() * Math.abs(m_driverController1.getLeftX()) * DriveConstants.DriverInputRedution, 0.06),
                MathUtil.applyDeadband(-m_driverController1.getRightX() * Math.abs(m_driverController1.getRightX()) * DriveConstants.DriverInputRedution, 0.06),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     new JoystickButton(m_driverController1, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


      //new JoystickButton(m_driverController,3).whileTrue(new ArmSetCMD(theArmSystem, 40));
     
      //new JoystickButton(m_driverController,4).whileTrue(new IntakeSetCMD(intakeSubsystem, .5));

     //new JoystickButton(m_driverController, Button.kCircle).whileTrue(new IntakeSetCMD(intakeSubsystem, false));

     
  }

   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommand() {
    /*
    // Placing the cone at start of auton
     public static boolean placeCone = true;             //took out public static may cause problems?
    //public final ArmSubsystem theArmSystem = new ArmSubsystem();
    //if(placeCone == true){
        //new ArmSetCMD(theArmSystem, 0, 57, -84);
       //if(Arm2Position == 57 && Arm3Position == -84){
           // new IntakeRunWheelsCMD(theIntakeSubsystem, -.3);
            //need to figure out how to run intake for 2 seconds
           // new ArmSetCMD(theArmSystem, 0,0, 0);
        //placeCone = false;
        
    
     

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory  exampleTrajectory;

   /*  if (AutoConstants.AutoRoute == 1){
            // An example trajectory to follow. All units in meters.
         exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(6, 0, new Rotation2d(0)), 
        config);
        public static boolean nextStep = true; */
                // need to measure correct distance to outside of community area

//we need to confirm both won't go at the same time!!!
/* 
        if(nextStep == true){
            // An example trajectory to follow. All units in meters.
         exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        List.of(new Translation2d(-.5, 0), new Translation2d(-1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(-1.75, 0, new Rotation2d(0)), 
        config);
        
         
    }else if (AutoConstants.AutoRoute == 2){
       //This Trajectory is just a place holder that moves nowhere
       exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                 List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),
                                                                 new Pose2d(6, 0, new Rotation2d(0)),config);


    }else if (AutoConstants.AutoRoute == 3){
        //This Trajectory is just a place holder that moves nowhere
         exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                 List.of(new Translation2d(0.01, 0), new Translation2d(0.01, 0)),
                                                                 new Pose2d(0.03, 0, new Rotation2d(0)),config);
    }else{
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                 List.of(new Translation2d(0.01, 0), new Translation2d(0.01, 0)),
                                                                 new Pose2d(0.03, 0, new Rotation2d(0)),config);
    }

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    //return new SequentialCommandGroup(new IntakeRunWheelsToPosition(theIntakeSubsystem, .4, 100), swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)));
  */}
}
