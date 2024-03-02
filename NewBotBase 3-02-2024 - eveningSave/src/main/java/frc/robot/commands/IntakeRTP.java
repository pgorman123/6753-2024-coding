package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRTP extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    private final double distance;
    private  double startPosition;

    public IntakeRTP(IntakeSubsystem intakeSubsystem, double speed, double distance) {
            this.distance = distance;
            this.intakeSubsystem = intakeSubsystem;
            this.speed = speed;
            addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

        startPosition = intakeSubsystem.CurrentIntakePosition();

    }

    @Override
    public void execute() {
        intakeSubsystem.runIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
       intakeSubsystem.runIntake(0); 
    }

    @Override
    public boolean isFinished() {
        
        if(speed > 0){
            if(intakeSubsystem.CurrentIntakePosition() >= startPosition+distance){
                return true;
            }else{
                return false;
            }
        }else if (speed < 0){
            if(intakeSubsystem.CurrentIntakePosition() <= startPosition-distance){
                return true;
            }else{
                return false;
            }
        }else{
            return true;
        }

    }
}
