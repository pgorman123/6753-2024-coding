package frc.robot.commands;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;


public class TestingIntakeRunWheelsCMD extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private String InOrOut;

    public TestingIntakeRunWheelsCMD(IntakeSubsystem intakeSubsystem, String InOrOut) {
            this.intakeSubsystem = intakeSubsystem;
            this.InOrOut = InOrOut;
            addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(InOrOut == "IN" ){
            intakeSubsystem.runIntake(IntakeConstants.IN); 
        }else if (InOrOut == "OUT"){
            intakeSubsystem.runIntake(IntakeConstants.OUT); 
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake(0); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
