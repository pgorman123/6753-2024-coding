package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterRunWheelsCMD extends Command {
    
    private final ShooterSubsystem ShooterSubsystem;
    private final double speed;

    public ShooterRunWheelsCMD(ShooterSubsystem ShooterSubsystem, double speed) {
            this.ShooterSubsystem = ShooterSubsystem;
            this.speed = speed;
            addRequirements(ShooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ShooterSubsystem.runShooter(speed);
    }

    @Override
    public void end(boolean interrupted) {
       ShooterSubsystem.runShooter(0); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
