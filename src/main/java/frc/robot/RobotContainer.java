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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.NewIntakeArmSetCMD;
import frc.robot.commands.ShooterArmSetCMD;
//import frc.robot.commands.AutoArmSetCMD;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.IntakeRTP;
import frc.robot.commands.IntakeRunWheelsCMD;
//import frc.robot.commands.LimelightCMD;
import frc.robot.commands.ClimberCMD;
import frc.robot.commands.ClimberLeftCMDtest;

import frc.robot.commands.TestingIntakeRunWheelsCMD;
import frc.robot.commands.TestingShooterRunWheelsCMD;

import frc.robot.commands.ShooterTipCMD;

import frc.robot.subsystems.NewIntakeArmSubsystem;
import frc.robot.subsystems.ShooterArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTipSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.LimelightHelpers;
//import frc.robot.subsystems.LimelightSubsystem;
import java.util.List;

public class RobotContainer {

    final DriveSubsystem m_robotDrive = new DriveSubsystem();

    XboxController m_driverController1 = new XboxController(2);
    XboxController m_driverController2 = new XboxController(OperatorConstants.kDriverController2Port);

    Joystick theJoystick = new Joystick(OperatorConstants.kDriverController1Port);

    //public final IntakeArmSubsystem theIntakeArmSystem = new IntakeArmSubsystem();
   // public final ShooterArmSubsystem theShooterArmSystem = new ShooterArmSubsystem();
      


    public final IntakeSubsystem theIntakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem theShooterSubsystem = new ShooterSubsystem();
    public final ShooterArmSubsystem theShooterArmSubsystem = new ShooterArmSubsystem();
    public final ShooterTipSubsystem theShooterTipSubsystem = new ShooterTipSubsystem();
    public final ClimberSubsystem theClimberSubsystem = new ClimberSubsystem();

    public final NewIntakeArmSubsystem theNewIntakeArmSubsystem = new NewIntakeArmSubsystem(); 

    public RobotContainer() {

        configureButtonBindings();
//The buttons are on the joystick. Trigger is "IN", side thumb button is "OUT". This has been tested

        //RUN INTAKE WHEELS
        new JoystickButton(theJoystick, 1)
            .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"));
        new JoystickButton(theJoystick, 2)
            .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT"));


        //RUN SHOOTER WHEELS
        //Inverted compared to the intake bc we think it should be due to the shooter being on the opposite side of the robot
        new JoystickButton(theJoystick, 7) //I put this on the joystick for the purpose of testing
            .whileTrue(new TestingShooterRunWheelsCMD(theShooterSubsystem, "OUT"));
        new JoystickButton(theJoystick, 8) //I put this on the joystick for the purpose of testing
            .whileTrue(new TestingShooterRunWheelsCMD(theShooterSubsystem, "IN"));


// This needs to be adjusted!! 

        //These buttons are to move the Intake Arm to a desired position, positive is down (towards floor)
        new JoystickButton(theJoystick, 11) //I put this on the joystick for the purpose of testing
            .whileTrue(new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, 0.011));
         //test position
            /* .whileTrue(
                if IntakeArmSetPosition is higher than Ground
                         new IntakeArmSetCMD(theIntakeArmSystem, = -0.25)); //test position
                then new IntakeArmSetCMD(theIntakeArmSystem, = )*/
        new JoystickButton(theJoystick, 12) //I put this on the joystick for the purpose of testing
            .whileTrue(new NewIntakeArmSetCMD(theNewIntakeArmSubsystem, .45)); //test position

//This needs to be adjusted!!

        //These buttons are to move the Shooter Arm to a desired position
        //Motor is 100 to 1
        //Alternate Encoder is on the bar
             //  new JoystickButton(theJoystick, 9) //I put this on the joystick for the purpose of testing
                   //    .whileTrue(new ShooterArmSetCMD(theShooterArmSubsystem, 0.1)); //random numbers. Needs adjustment
             //   new JoystickButton(theJoystick, 10) //I put this on the joystick for the purpose of testing
                     //   .whileTrue(new ShooterArmSetCMD(theShooterArmSubsystem, -0.01)); //random numbers. Needs adjustment


           
        //Low Cube Place
//This needs to be adjusted!!

        //These buttons are to move ShooterTip to desired position
        //The two postions are shoot and place in Amp
       new JoystickButton(theJoystick, 3) //I put this on the joystick for the purpose of testing
                .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .04)); //random numbers. Needs adjustment

        new JoystickButton(theJoystick, 4) //I put this on the joystick for the purpose of testing
                .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .5)); //random numbers. Needs adjustment
                
         //new JoystickButton(theJoystick, 5) //I put this on the joystick for the purpose of testing
             //   .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .4));
        

//Climber up on driverstation buttons
        new JoystickButton(m_driverController2,3)
                .whileTrue(new ClimberCMD(theClimberSubsystem, 2,2));

//Climber down on driverstation buttons
         new JoystickButton(m_driverController2,6)
                .whileTrue(new ClimberCMD(theClimberSubsystem, 0.1,0.1));

//LeftClimberUP 
      //   new JoystickButton(theJoystick,6)
         //       .whileTrue(new ClimberLeftCMDtest(theClimberSubsystem,5 ));

//LeftClimberDown 
       //  new JoystickButton(theJoystick,5)
            //    .whileTrue(new ClimberLeftCMDtest(theClimberSubsystem,0.1 ));

                
//New Intergrated Scoring Button. This is commented out bc we do not know how to put 2 commands on 1 button AB

//Shooter
      //  new JoystickButton(theJoystick, 5). //I put this on the joystick for the purpose of testing
        
             //  .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .26));
            //    .whileTrue(new TestingShooterRunWheelsCMD(theShooterSubsystem, "IN")); 
        
                
//Amp
       // new JoystickButton(theJoystick, 6) //I put this on the joystick for the purpose of testing
              //  .whileTrue(new ShooterTipCMD(theShooterTipSubsystem, .5)); //Likely we need this number at one, but we set it lower for testing AB

                



        // - Arm positions - // 2023 Code Buttons 

      /*   // Return to Zero
        new JoystickButton(m_driverController2, OperatorConstants.ArmReset)
                .whileTrue(new ArmSetCMD(theArmSystem, 0, 0));
        // Pick up Cube and Place Ground
        new JoystickButton(m_driverController2, OperatorConstants.CubePickup)
                .whileTrue(new ArmSetCMD(theArmSystem, .05, 0.27));
        // Low Cube Place
        new JoystickButton(m_driverController2, OperatorConstants.CubePlaceLow)
                .whileTrue(new ArmSetCMD(theArmSystem, .019, 0.017));
        // Medium Cube Place
        new JoystickButton(m_driverController2, OperatorConstants.CubePlaceMid)
                .whileTrue(new ArmSetCMD(theArmSystem, .135, 0.064));
        // High Cube Place
        new JoystickButton(m_driverController2, OperatorConstants.CubePlaceHigh)
                .whileTrue(new ArmSetCMD(theArmSystem, .241, .224));
        // High Cone place
        new JoystickButton(m_driverController2, OperatorConstants.ConePlaceHigh)
                .whileTrue(new ArmSetCMD(theArmSystem, .270, .454));
        // Medium Cone Place
        new JoystickButton(m_driverController2, OperatorConstants.ConePlaceMid)
                .whileTrue(new ArmSetCMD(theArmSystem, .302, .608));
        // Ground Cone Place
        new JoystickButton(m_driverController2, OperatorConstants.ConePlaceLow)
                .whileTrue(new ArmSetCMD(theArmSystem, 0, 0.135));
        // Human Player Station
        new JoystickButton(m_driverController2, OperatorConstants.ConePickup)
                .whileTrue(new ArmSetCMD(theArmSystem, 0.00, .083));

        // ----------
    */


    // 2023 Intake run commands, on Joystick
    /*
        new JoystickButton(theJoystick, 1)
                .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "IN"));
        new JoystickButton(theJoystick, 2)
                .whileTrue(new TestingIntakeRunWheelsCMD(theIntakeSubsystem, "OUT"));

        new JoystickButton(theJoystick, 11)
                .whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.75));

        new JoystickButton(theJoystick, 9)
                .whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));

        new JoystickButton(theJoystick, 10)
                .whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, -.6));

        new JoystickButton(theJoystick, 12)
                .whileTrue(new IntakeRunWheelsCMD(theIntakeSubsystem, .75));
        // ------------------------------------------------------------------------------------------------------------
*/
        m_robotDrive.setDefaultCommand(

                new RunCommand(() -> m_robotDrive.drive(
                        MathUtil.applyDeadband(-theJoystick.getY()
                                * (-theJoystick.getRawAxis(3) + 1) / 2,
                                0.1),
                        MathUtil.applyDeadband(-theJoystick.getX()
                                * (-theJoystick.getRawAxis(3) + 1) / 2,
                                0.1),
                        MathUtil.applyDeadband(-theJoystick.getZ()
                                * (-theJoystick.getRawAxis(3) + 1) / 2,
                                0.2),
                        true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 

        

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)

                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory Trajectory;
        Trajectory TrajectoryA;
        Trajectory TrajectoryB;
        Trajectory TrajectoryC;
        Trajectory TrajectoryD;
        Trajectory TrajectoryE;

 if (AutoConstants.AutoRoute == 234){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1.5, 0)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, -.4)
                        ),
                                new Pose2d(5.5, -.7, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, -.4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, .4)
                        ),
                                new Pose2d(5.5, .7, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, .7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, .4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                return new SequentialCommandGroup(
                
                    Move(TrajectoryA),
                     Move(TrajectoryB),
                     Move(TrajectoryC),
                     Move(TrajectoryD),
                     Move(TrajectoryE)
                    

            );
                
        
    
    }else if (AutoConstants.AutoRoute == 233){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1.5, 0)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(6.3, -.7, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(6.3, -.7, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                return new SequentialCommandGroup(
                
                    Move(TrajectoryA),
                     Move(TrajectoryB),
                     Move(TrajectoryC)
                     
                 
                    

            );
     } else if (AutoConstants.AutoRoute == 232){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1.5, 0)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, .4)
                        ),
                                new Pose2d(6.3, .7, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, .7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, .4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(6.3, -.7, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                return new SequentialCommandGroup(
                
                    Move(TrajectoryA)
                     
                    
                    

            );
     }else if (AutoConstants.AutoRoute == 231){/*Red Auto 1 2024 */

        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1.5, 0)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, .4)
                        ),
                                new Pose2d(6.3, .7, new Rotation2d(0)),config);

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, .7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, .4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);
        TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(2.9, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(6.3, -.7, new Rotation2d(0)),config);

         TrajectoryE = TrajectoryGenerator.generateTrajectory(new Pose2d(6.3, -.7, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(5, -.4)
                        ),
                                new Pose2d(2.9, 0, new Rotation2d(0)),config);


                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                return new SequentialCommandGroup(
                //aim and fire
                
                    
                    

            );
      } else if (AutoConstants.AutoRoute == 224){/*Red Auto 2 2024 */
        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1.1, 0)
                        ),
                                new Pose2d(2, 0, new Rotation2d(0)),config);

        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(1, -0.8)
                        ),
                                new Pose2d(2, -1.5, new Rotation2d(0)),config);

        /*TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(2, -1.5, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(2.5, 0)
                        ),
                                new Pose2d(3, -2, new Rotation2d(0)),config); */

         TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(2, -1.5, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, 0)
                        ),
                                new Pose2d(5.5, 0, new Rotation2d(0)),config);

          TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(4.2, 0)
                        ),
                                new Pose2d(2.2, 0, new Rotation2d(0)),config);

                

                m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());
         


                return new SequentialCommandGroup(
                
                    Move(TrajectoryA),
                    Move(TrajectoryB),
                    Move(TrajectoryC),
                    Move(TrajectoryD)
                  

            );


    
    }else if (AutoConstants.AutoRoute == 213){/*Red Auto 3 2024 */
        TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(  
                            new Translation2d(3.1, 0)
                        ),
                                new Pose2d(5.5, -1, new Rotation2d(0)),config);
        TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, -1, new Rotation2d(0)),
                        List.of(  
                                new Translation2d(3.5, -0.5)
                        ),
                            new Pose2d(2, 0, new Rotation2d(0)),config);
        TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 0, new Rotation2d(0)),
                        List.of(  
                                new Translation2d(3.5, 0.5)
                        ),
                            new Pose2d(5.5, 1, new Rotation2d(0)),config);

         TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, 1, new Rotation2d(0)),
                        List.of(  
                                new Translation2d(3.5, 0.5)
                        ),
                            new Pose2d(2, 0, new Rotation2d(0)),config);
         
            m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


            return new SequentialCommandGroup(
                
                    Move(TrajectoryA),
                     Move(TrajectoryB),
                      Move(TrajectoryC),
                       Move(TrajectoryD)
            

            );

        }else if (AutoConstants.AutoRoute == 134){/*Blue Auto 1 2024 */
                TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                            List.of( 

                                 new Translation2d(3.8, 0.5)

                            ),
                            new Pose2d(5.5, 1, new Rotation2d(0)),config);

                TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, 1, new Rotation2d(0)),
                            List.of( 

                                 new Translation2d(4.2, .5)

                            ),
                            new Pose2d(3.7, 0, new Rotation2d(0)),config);

                TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(3.7, 0, new Rotation2d(0)),
                            List.of( 

                                 new Translation2d(4.2, -.5)

                            ),
                            new Pose2d(5.5, -1, new Rotation2d(0)),config);

                TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, -1, new Rotation2d(0)),
                            List.of( 

                                 new Translation2d(4.2, -0.5)

                            ),
                            new Pose2d(3.7, 0, new Rotation2d(0)),config);

                    m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

                            return new SequentialCommandGroup(
                Move(TrajectoryA),
                Move(TrajectoryB),
                Move(TrajectoryC),
                Move(TrajectoryD)
                
                );



                                     

        
        
        }else if (AutoConstants.AutoRoute == 124){/*Blue Auto 2 2024 */
            TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                            List.of(
                            
                            new Translation2d(.4, 0)),
                            new Pose2d(.8, 0, new Rotation2d(0)),config);

            TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(.8, 0, new Rotation2d(0)),
                            List.of(
                            
                            new Translation2d(0, -.6)),
                            new Pose2d(0.8, -2.2, new Rotation2d(0)),config);


            m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());


                              return new SequentialCommandGroup(

                                    Move(TrajectoryA),
                                    Move(TrajectoryB)

                              );
                    
        
        }else if (AutoConstants.AutoRoute == 113){/*Blue Auto 3 2024  Needs work to go backwards*/
            TrajectoryA = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
                            List.of( 

                                 new Translation2d(0.8, 0)

                            ),
                            new Pose2d(1.6, 0, new Rotation2d(0)),config);

            TrajectoryB = TrajectoryGenerator.generateTrajectory(new Pose2d(1.6, 0, new Rotation2d(0)),
                            List.of(new Translation2d(3.3,-0.2)),
                            new Pose2d(5.5, -0.5, new Rotation2d(0)),config);

            TrajectoryC = TrajectoryGenerator.generateTrajectory(new Pose2d(5.5, -0.5, new Rotation2d(0)),
                            List.of(new Translation2d(4.2,0)),
                            new Pose2d(3.7, 0, new Rotation2d(0)),config);

             /* TrajectoryD = TrajectoryGenerator.generateTrajectory(new Pose2d(5, 0, new Rotation2d(0)),
                            List.of(new Translation2d(8.3,0)),
                            new Pose2d(2, 0, new Rotation2d(0)),config); */


             m_robotDrive.resetOdometry(TrajectoryA.getInitialPose());

             return new SequentialCommandGroup(
                Move(TrajectoryA),
                Move(TrajectoryB),
                Move(TrajectoryC)
                //Move(TrajectoryD)

             );

       /*  if (AutoConstants.AutoRoute == 1) {

            Trajectory = TrajectoryGenerator.generateTrajectory(

                    new Pose2d(0, 0, new Rotation2d(0)),

                    List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),

                    new Pose2d(5, 0, new Rotation2d(0)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));*/
/*
        } else if (AutoConstants.AutoRoute == 2) {
            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(0.5, 0), new Translation2d(3, 0)),
                    new Pose2d(AutoConstants.AutoDistance, 0, new Rotation2d(0)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .22, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 5), Move(Trajectory));

        } else if (AutoConstants.AutoRoute == 3) {
            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(0.75, 0), new Translation2d(1.5, 0)),
                    new Pose2d(1.95, 0, new Rotation2d(3.2)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .22, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 5),
                    new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory),
                    new AutoBalance(m_robotDrive));

        } else if (AutoConstants.AutoRoute == 85) {
            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(0.75, 0), new Translation2d(3.5, 0),
                            new Translation2d(2.5, 0)),
                    new Pose2d(0, 0, new Rotation2d(3.2)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .22, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 5),
                    new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory),
                    new AutoBalance(m_robotDrive));

        } else if (AutoConstants.AutoRoute == 86) {

            Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(

                    new Pose2d(0, 0, new Rotation2d(3.2)),

                    List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),

                    new Pose2d(2, 0, new Rotation2d(3.2)), config);

            Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(

                    new Pose2d(2, 0, new Rotation2d(.1)),

                    List.of(new Translation2d(1.9, 0), new Translation2d(1.8, 0)),

                    new Pose2d(1.7, 0, new Rotation2d(.1)), config);

            Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(

                    new Pose2d(1.7, 0, new Rotation2d(.1)),

                    List.of(new Translation2d(1, .3), new Translation2d(.5, .4)),

                    new Pose2d(0, .5, new Rotation2d(.1)), config);

            m_robotDrive.resetOdometry(trajectory1.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .22, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 5),
                    new AutoArmSetCMD(theArmSystem, 0, 0), Move(trajectory1),
                    Move(trajectory2), Move(trajectory3), new AutoBalance(m_robotDrive));

        } else if (AutoConstants.AutoRoute == 4) {

            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(0.1, 0), new Translation2d(2, 0)),
                    new Pose2d(6, 0, new Rotation2d(0)), config);
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .22, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 5),
                    new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory),
                    new AutoBalance(m_robotDrive));

        } else if (AutoConstants.AutoRoute == 5) {

            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(3.15, 0), new Translation2d(3.15, 1.4)),
                    new Pose2d(1.75, 1.4, new Rotation2d(3.2)), config);
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .241, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 4),
                    new ParallelCommandGroup(
                            new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory)),
                    new AutoBalance(m_robotDrive))
                    .withTimeout(14.9)
                    .andThen(m_robotDrive::setX);

        } else if (AutoConstants.AutoRoute == 6) {

            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(3.2)),
                    List.of(new Translation2d(3.15, 0), new Translation2d(3.15, -1.4)),
                    new Pose2d(1.75, -1.4, new Rotation2d(3.2)), config);
            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return new SequentialCommandGroup(
                    new AutoArmSetCMD(theArmSystem, .241, .224),
                    new IntakeRTP(theIntakeSubsystem, 1, 4),
                    new ParallelCommandGroup(
                            new AutoArmSetCMD(theArmSystem, 0, 0), Move(Trajectory)),
                    new AutoBalance(m_robotDrive))
                    .withTimeout(14.9)
                    .andThen(m_robotDrive::setX);
*/
        } else {

            Trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    List.of(new Translation2d(0.01, 0), new Translation2d(0.01, 0)),
                    new Pose2d(0.03, 0, new Rotation2d(0)), config);

            m_robotDrive.resetOdometry(Trajectory.getInitialPose());

            return Move(Trajectory).andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        }
    }

    SwerveControllerCommand Move(Trajectory newPath) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(newPath,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,

                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
                m_robotDrive::setModuleStates, m_robotDrive);
    }
}
