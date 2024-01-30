package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;



public final class LimelightCMD extends Command {

    private final LimelightSubsystem limelightSubsystem;

    public LimelightCMD(LimelightSubsystem subsystem) {
        this.limelightSubsystem = subsystem;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("[LL-CMD] Limelight Command Started");
    }

    @Override
    public void execute() {
        limelightSubsystem.sendLimelightValues();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[LL-CMD] Limelight Command Ended");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
