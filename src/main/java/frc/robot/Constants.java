// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    //2023 Code that sets button numbers
    public static final int kDriverController1Port = 0;
    public static final int kDriverController2Port = 1;

    public static final int ConePlaceHigh = 1;
    public static final int ConePlaceMid  = 2;
    public static final int ConePlaceLow  = 3;

    public static final int CubePlaceHigh = 4;
    public static final int CubePlaceMid  = 5;
    public static final int CubePlaceLow  = 6;

    public static final int CubePickup = 8;
    public static final int ConePickup = 9;

    public static final int ArmReset = 7;

    public static final int ConeIntake = 2;
    public static final int ConeOutake = 4;

    public static final int CubeIntake = 1;
    public static final int CubeOutake = 3;
    public static final String AutoBalance = null;

    //2024 Code that sets button numbers
    /*public static final int RingShooterIn = 8;
    public static final int RingShooterOut = 9;
    public static final int IntakeArmToShooter = 1;
    public static final int IntakeArmPickupPosition = 4;
    public static final int ShooterArmUnderChain = 6;
    public static final int ShooterArmShootPosition = 3;
    */
  }

  public static class ArmContants{
    public static final int IntakeArmCANID = 10;
    //public static final int ShooterArmFCANID = 11;
    public static final int ShooterArmCANID = 11;

    //These will need to be adjusted!!
    //Not sure if we are using these limits.
    public static final double ShooterArmtoplimit = 95;
    public static final double ShooterArmbottomlimit = -2;
    public static final double IntakeArmtoplimit = 2;
    public static final double IntakeArmbottomlimit = 90;

    //This is used for determining whether or not to move Arm 1;

    //We commented this out when copying code. May need this later!
    //public static final double ShooterArmSafeThreshold = 5; //old number was 17
  
  }
public static class ClimberConstants{

  public static final int ClimberLeftCANID = 16;
  public static final int ClimberRightCANID = 17;
}
  public static final class ShooterConstants{
    //These are CAN ID's
    public static final int ShooterTip = 13; //we put this in to allow the code to work, without a real id; 
    public static final int ShooterF = 14;
    public static final int Shooter = 15; 

    //public static final shooterTipAmpShotPos = 

    public static double IN;
    public static double OUT;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    //public static final double kMaxSpeedMetersPerSecond = 4.8;
    //public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

// Joystick speed control, reduces driver control 
public static final double DriverInputRedution = 1; 

    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5); //23.25 old number (3/02/24) AB
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5); //28.5 old number (3/02/24) AB
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians 
    //The zero point is the front right therefore all of the other angles are 90 degrees and 180 degrees off. 
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0; 
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX DRIVE TRAIN CAN IDs
    
    //Driving
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;
    //Turning
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;
    
    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.25;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static double AutoRoute;
    public static double AutoDistance;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants {

    //We commented these out bc they were either duplicates or going to be used else where. AB
    //public static final int IntakeTogglePort = 14;
    //public static final int IntakeLeftPort = 12;
    //public static final int IntakeRightPort = 13;
    public static final int Intake = 9; //Can ID
      
    public static final double IntakeOpen = 0;
    public static final double IntakeClose = 16;

    //upper old is -0.25
    //lower old is 0.45

    //These limits were set 2/26/24  AB
    //public static final double IntakeArmUpperLimit = 0;
    //public static final double IntakeArmLowerLimit = 0.33;

    public static double IN;
    public static double OUT;
  }

}
