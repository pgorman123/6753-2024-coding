package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTipSubsystem;

public class ShooterTipCMD extends Command {

    private ShooterTipSubsystem shootertipSubsystem;   
    private double DesiredShooterTipPosition;
 

    public ShooterTipCMD(ShooterTipSubsystem shootertipSubsystem, double ShooterTipPosition){
        this.DesiredShooterTipPosition = ShooterTipPosition;
        

        this.shootertipSubsystem = shootertipSubsystem;
        addRequirements(shootertipSubsystem);
    }

    public ShooterTipCMD(ShooterTipSubsystem theShooterTIpSystem, int i, int j, Translation2d translation2d, Rotation2d rotation2d,
            Translation2d translation2d2, Rotation2d rotation2d2, ShooterTipCMD shootertipCMD,
            IntakeRunWheelsCMD intakeRunWheelsCMD, IntakeRunWheelsCMD intakeRunWheelsCMD2, ShooterTipCMD shootertipCMD2,
            Translation2d translation2d3, Rotation2d rotation2d3, TrajectoryConfig config) {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shootertipSubsystem.setShooterTipPosition(DesiredShooterTipPosition);
       
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}

