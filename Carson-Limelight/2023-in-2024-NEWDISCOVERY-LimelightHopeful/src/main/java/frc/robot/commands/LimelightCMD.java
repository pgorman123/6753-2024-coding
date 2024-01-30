package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class LimelightCMD extends Command {

    public LimelightSubsystem limelight_Subsystem;

    public LimelightCMD() {
        
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Limelight X", limelight_Subsystem.x);
        SmartDashboard.putNumber("Limelight Y", limelight_Subsystem.y);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
