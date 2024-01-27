package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutonArmCMD extends Command {

    private ArmSubsystem armSubsystem;   
    private double DesiredArm2Position;
    private double DesiredArm3Position;

    public AutonArmCMD(ArmSubsystem armSubsystem, double Arm2Position, double Arm3Position ){
        this.DesiredArm2Position = Arm2Position;
        this.DesiredArm3Position = Arm3Position;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.setArm2Position(DesiredArm2Position);
        armSubsystem.setArm3Position(DesiredArm3Position);
    }

    @Override
    public void end(boolean interrupted) {
        //armSubsystem.setArm2Position(0);
        //armSubsystem.setArm3Position(0);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(DesiredArm2Position - armSubsystem.CurrentArm2AltPos()) < .01 && Math.abs(DesiredArm3Position - armSubsystem.CurrentArm3AltPos()) < .01) {
            return true;
        } else {
            return false;
        }
        
    }


}

