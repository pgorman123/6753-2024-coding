package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LimelightSubsystem extends SubsystemBase {
// 0 1 5 6
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static double[] botPose = table.getEntry("botpose[0]").getDoubleArray(new double[0]);

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");
    public static NetworkTableEntry tid = table.getEntry("tid");

    public static double x = tx.getDouble(0.0);
    public static double y = ty.getDouble(0.0);
    public static double AprilTagID = tid.getDouble(0.0);

    public LimelightSubsystem() {}

    public void sendLimelightValues() {
        LimelightSubsystem.table = NetworkTableInstance.getDefault().getTable("limelight");
        LimelightSubsystem.tx = table.getEntry("tx");
        LimelightSubsystem.ty = table.getEntry("ty");
        LimelightSubsystem.ta = table.getEntry("ta");
        LimelightSubsystem.tid = table.getEntry("tid");

        LimelightSubsystem.x = tx.getDouble(0.0);
        LimelightSubsystem.y = ty.getDouble(0.0);
        LimelightSubsystem.AprilTagID = tid.getDouble(0.0);

        SmartDashboard.putNumber("Limelight X", LimelightSubsystem.x);
        SmartDashboard.putNumber("Limelight Y", LimelightSubsystem.y);
        SmartDashboard.putNumber("Limelight AprilTag ID", LimelightSubsystem.AprilTagID);

        SmartDashboard.putNumber("Limelight BotPose 0", botPose[0]);


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight X", LimelightSubsystem.x);
        SmartDashboard.putNumber("Limelight Y", LimelightSubsystem.y);
        SmartDashboard.putNumber("Limelight AprilTag ID", LimelightSubsystem.AprilTagID);

       /* SmartDashboard.putNumber("Limelight BotPose 0", botPose); */
    }

}

//put periodic above sendLimelightValues if not working