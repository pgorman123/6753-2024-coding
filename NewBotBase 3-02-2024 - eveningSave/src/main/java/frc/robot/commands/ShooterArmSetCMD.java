package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterArmSubsystem;

public class ShooterArmSetCMD extends Command {

    private ShooterArmSubsystem shooterarmSubsystem;   
    private double DesiredShooterArmPosition;
    
    public ShooterArmSetCMD(ShooterArmSubsystem shooterarmSubsystem, double ShooterArmPosition){
        this.DesiredShooterArmPosition = ShooterArmPosition;
        
        this.shooterarmSubsystem = shooterarmSubsystem;
        addRequirements(shooterarmSubsystem);
    }

    public ShooterArmSetCMD(ShooterArmSubsystem theShooterArmSystem, int i, int j, Translation2d translation2d, Rotation2d rotation2d,
            Translation2d translation2d2, Rotation2d rotation2d2, ShooterArmSetCMD shooterarmSetCMD,
            IntakeRunWheelsCMD intakeRunWheelsCMD, IntakeRunWheelsCMD intakeRunWheelsCMD2, ShooterArmSetCMD shooterarmSetCMD2,
            Translation2d translation2d3, Rotation2d rotation2d3, TrajectoryConfig config) {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterarmSubsystem.setShooterArmPosition(DesiredShooterArmPosition);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}

