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

//read values periodically
public double x = tx.getDouble(0.0);
public double y = ty.getDouble(0.0);


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
    }

    
}