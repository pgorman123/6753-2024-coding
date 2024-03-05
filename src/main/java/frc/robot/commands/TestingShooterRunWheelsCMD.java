package frc.robot.commands;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;


public class TestingShooterRunWheelsCMD extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private String InOrOut;

    public TestingShooterRunWheelsCMD(ShooterSubsystem shooterSubsystem, String InOrOut) {
            this.shooterSubsystem = shooterSubsystem;
            this.InOrOut = InOrOut;
            addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(InOrOut == "IN" ){
            shooterSubsystem.runShooter(ShooterConstants.IN); 
        }else if (InOrOut == "OUT"){
            shooterSubsystem.runShooter(ShooterConstants.OUT); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.runShooter(0); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
