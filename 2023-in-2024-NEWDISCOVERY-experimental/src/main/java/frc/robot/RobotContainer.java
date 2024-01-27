// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
// import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmSetCMD;
import frc.robot.commands.AutoArmSetCMD;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.IntakeRTP;
import frc.robot.commands.IntakeRunWheelsCMD;
import frc.robot.commands.TestingIntakeRunWheelsCMD;
//import frc.robot.commands.IntakeRunWheelsToPosition;
// import frc.robot.commands.IntakeToggleCMD;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.IntakeSetCMD;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


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
 //XboxController m_driverController1 = new XboxController(OperatorConstants.kDriverController1Port);
  XboxController m_driverController1 = new XboxController(2);
  XboxController m_driverController2 = new XboxController(OperatorConstants.kDriverController2Port);

  Joystick theJoystick = new Joystick(OperatorConstants.kDriverController1Port);

  //Define an arm subsystem
  public final ArmSubsystem theArmSystem = new ArmSubsystem();

  //Define an intake subsystem
  public final IntakeSubsystem theIntakeSubsystem = new IntakeSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

     //Arm positions
        //Return to Zero 
    new JoystickButton(m_driverController2,OperatorConstants.ArmReset).whileTrue(new ArmSetCMD(theArmSystem, 0, 0));
        //Pick up Cube and Place Ground 
    new JoystickButton(m_driverController2,OperatorConstants.CubePickup).whileTrue(new ArmSetCMD(theArmSystem,.05 , 0.27));
        //Low Cube Place
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceLow).whileTrue(new ArmSetCMD(theArmSystem, .019, 0.017 ));
        //Medium Cube Place 
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceMid).whileTrue(new ArmSetCMD(theArmSystem, .135, 0.064));
        //High Cube Place 
    new JoystickButton(m_driverController2, OperatorConstants.CubePlaceHigh).whileTrue(new ArmSetCMD(theArmSystem, .241, .224));
         //High Cone place
    new JoystickButton(m_driverController2,OperatorConstants.ConePlaceHigh).whileTrue(new ArmSetCMD(theArmSystem,.270, .454));
        //Medium Cone Place 
    new JoystickButton(m_driverController2, OperatorConstants.ConePlaceMid).whileTrue(new ArmSetCMD(theArmSystem, .302, .608));
        //Ground Cone Place 
    new JoystickButton(m_driverController2, OperatorConstants.ConePlaceLow).whileTrue(new ArmSetCMD(theArmSystem, 0, 0.135));
        //Human Player Station
    new JoystickButton(m_driverController2,OperatorConstants.ConePickup).whileTrue(new ArmSetCMD(theArmSystem,0.00, .083));

    //------------------------------------------------------------------------------------------------------------
    /* 
        //Intake Out Cube
    new JoystickButton(m_driverController1,OperatorConstants.CubeOutake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));
        //Intake In Cube
    new JoystickButton(m_driverController1,OperatorConstants.CubeIntake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
        //Intake In Cone
    new JoystickButton(m_driverController1,OperatorConstants.ConeIntake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));
        //Intake Out Cone
    new JoystickButton(m_driverController1,OperatorConstants.ConeOutake).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75)); 
        */
    //------------------------------------------------------------------------------------------------------------



    new JoystickButton(theJoystick,1).whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"));
    new JoystickButton(theJoystick,2).whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT"));


        
    new JoystickButton(theJoystick,11).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));
       
    new JoystickButton(theJoystick,9).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
        
    new JoystickButton(theJoystick,10).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));
        
    new JoystickButton(theJoystick,12).whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
    //------------------------------------------------------------------------------------------------------------
      
    //new JoystickButton(theJoystick, 7).whileTrue(new AutoBalance(m_robotDrive));



    //new JoystickButton(theJoystick,8).onTrue(new IntakeRTP(theIntakeSubsystem, .75, 5));
    //new JoystickButton(theJoystick,7).onTrue(new IntakeRTP(theIntakeSubsystem, -.5, 5));


    //m_driverController.a().whileTrue(new testcmd());
    // Configure default commands


    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        /*new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController1.getLeftY() * Math.abs(m_driverController1.getLeftY()) * DriveConstants.DriverInputRedution, 0.06),
                MathUtil.applyDeadband(-m_driverController1.getLeftX() * Math.abs(m_driverController1.getLeftX()) * DriveConstants.DriverInputRedution, 0.06),
                MathUtil.applyDeadband(-m_driverController1.getRightX() * Math.abs(m_driverController1.getRightX()) * DriveConstants.DriverInputRedution, 0.06),
                true),
            m_robotDrive)); */

            new RunCommand(
                () -> m_robotDrive.drive(
                    MathUtil.applyDeadband(-theJoystick.getY() * (-theJoystick.getRawAxis(3) + 1) / 2, 0.1),
                    MathUtil.applyDeadband(-theJoystick.getX() * (-theJoystick.getRawAxis(3) + 1) / 2, 0.1),
                    MathUtil.applyDeadband(-theJoystick.getZ() * (-theJoystick.getRawAxis(3) + 1) / 2, 0.2),
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
     //new JoystickButton(m_driverController1, Button.kR1.value)
       // .whileTrue(new RunCommand(
          //  () -> m_robotDrive.setX(),
            //m_robotDrive));


      //new JoystickButton(m_driverController,3).whileTrue(new ArmSetCMD(theArmSystem, 40));
     
      //new JoystickButton(m_driverController,4).whileTrue(new IntakeSetCMD(intakeSubsystem, .5));

     //new JoystickButton(m_driverController, Button.kCircle).whileTrue(new IntakeSetCMD(intakeSubsystem, false));

     
    }

   
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory  Trajectory;

        if (AutoConstants.AutoRoute == 1){
                // An example trajectory to follow. All units in meters.
            Trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.15, 0, new Rotation2d(0)), 
            config);
          
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));

            
        }else if (AutoConstants.AutoRoute == 2){
        Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
                                                                    List.of(new Translation2d(0.5, 0), new Translation2d(3, 0)),
                                                                    new Pose2d(AutoConstants.AutoDistance, 0, new Rotation2d(0)),config);
            
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .22, .224), new IntakeRTP(theIntakeSubsystem, 1, 5 ),
                                             Move(Trajectory));
                                                        

        }else if (AutoConstants.AutoRoute == 3){
            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
                                                                    List.of(new Translation2d(0.75, 0), new Translation2d(1.5, 0)),
                                                                    new Pose2d(1.95, 0, new Rotation2d(3.2)),config);
              //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .22, .224),
                                                new IntakeRTP(theIntakeSubsystem, 1, 5 ), 
                                                new AutoArmSetCMD(theArmSystem, 0, 0),
                                                Move(Trajectory),
                                                new AutoBalance(m_robotDrive) );

        }else if (AutoConstants.AutoRoute == 85){
            //similar to 3 but goes over and back
            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
                                                                    List.of(new Translation2d(0.75, 0), new Translation2d(3.5, 0), new Translation2d(2.5, 0)),
                                                                    new Pose2d(1.95, 0, new Rotation2d(3.2)),config);
                                                                    
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .22, .224),
                                                new IntakeRTP(theIntakeSubsystem, 1, 5 ), 
                                                new AutoArmSetCMD(theArmSystem, 0, 0),
                                                Move(Trajectory),
                                                new AutoBalance(m_robotDrive) );

                                                        
        }else if (AutoConstants.AutoRoute == 4){
            //
            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
            List.of(new Translation2d(0.1, 0), new Translation2d(2, 0)),
            new Pose2d(6, 0, new Rotation2d(0)),config);
            
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .22, .224),
                                                new IntakeRTP(theIntakeSubsystem, 1, 5 ), 
                                                new AutoArmSetCMD(theArmSystem, 0, 0),
                                                Move(Trajectory),
                                                new AutoBalance(m_robotDrive) );

        }else if (AutoConstants.AutoRoute == 5){
            //left auton that places and ballances
           // Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
           //                                                         List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),
           //                                                         new Pose2d(1.5, 0, Rotation2d.fromDegrees(180)),config);

            //Trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
              //                                                          new Pose2d(1, 0, Rotation2d.fromDegrees(180)),
                //                                                        new Pose2d(1.95,0,Rotation2d.fromDegrees(180))), config);
            

            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
                                                        List.of(new Translation2d(3.15, 0), new Translation2d(3.15, 1.4)),
                                                                new Pose2d(1.75, 1.4, new Rotation2d(3.2)),config);
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .241, .224),
                                                new IntakeRTP(theIntakeSubsystem, 1, 4 ),
                                                new ParallelCommandGroup(new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory)), 
                                                new AutoBalance(m_robotDrive) ).withTimeout(14.9).andThen(m_robotDrive::setX);
       
         }else if (AutoConstants.AutoRoute == 6){
            //left auton that places and ballances
           // Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
           //                                                         List.of(new Translation2d(0.5, 0), new Translation2d(1, 0)),
           //                                                         new Pose2d(1.5, 0, Rotation2d.fromDegrees(180)),config);

            //Trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
              //                                                          new Pose2d(1, 0, Rotation2d.fromDegrees(180)),
                //                                                        new Pose2d(1.95,0,Rotation2d.fromDegrees(180))), config);
            

            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(3.2)),
                                                        List.of(new Translation2d(3.15, 0), new Translation2d(3.15, -1.4)),
                                                                new Pose2d(1.75, -1.4, new Rotation2d(3.2)),config);
            //m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return new SequentialCommandGroup(new AutoArmSetCMD(theArmSystem, .241, .224),
                                                new IntakeRTP(theIntakeSubsystem, 1, 4 ),
                                                new ParallelCommandGroup(new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory)), 
                                                new AutoBalance(m_robotDrive) ).withTimeout(14.9).andThen(m_robotDrive::setX);

        }else{
                        //This Trajectory is just a place holder that moves forward but only barely
            Trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                                                                    List.of(new Translation2d(0.01, 0), new Translation2d(0.01, 0)),
                                                                    new Pose2d(0.03, 0, new Rotation2d(0)),config);

           // m_robotDrive.zeroHeading();
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());
            
            return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));

        }

       

        

        // Run path following command, then stop at the end.
        //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        //return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        //return new SequentialCommandGroup(new IntakeRTP(theIntakeSubsystem, 1, 5 ), Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false)));



    }

    SwerveControllerCommand Move(Trajectory newPath) {

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

       return new SwerveControllerCommand(
            newPath,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    }

}

