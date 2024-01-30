package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class LimelightSubsystem extends SubsystemBase {

    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static NetworkTableEntry tx = table.getEntry("tx");
    public static NetworkTableEntry ty = table.getEntry("ty");
    public static NetworkTableEntry ta = table.getEntry("ta");

    public static double x = tx.getDouble(0.0);
    public static double y = ty.getDouble(0.0);

    public LimelightSubsystem() {}

    public void sendLimelightValues() {
        LimelightSubsystem.table = NetworkTableInstance.getDefault().getTable("limelight");
        LimelightSubsystem.tx = table.getEntry("tx");
        LimelightSubsystem.ty = table.getEntry("ty");
        LimelightSubsystem.ta = table.getEntry("ta");

        LimelightSubsystem.x = tx.getDouble(0.0);
        LimelightSubsystem.y = ty.getDouble(0.0);

        SmartDashboard.putNumber("Limelight X", LimelightSubsystem.x);
        SmartDashboard.putNumber("Limelight Y", LimelightSubsystem.y);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight X", LimelightSubsystem.x);
        SmartDashboard.putNumber("Limelight Y", LimelightSubsystem.y);
    }


}

//put periodic above sendLimelightValues if not working