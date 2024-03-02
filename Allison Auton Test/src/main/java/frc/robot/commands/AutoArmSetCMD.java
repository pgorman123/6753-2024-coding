package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmSetCMD extends Command {

    private ArmSubsystem armSubsystem;   
    private double DesiredArm2Position;
    private double DesiredArm3Position;

    public AutoArmSetCMD(ArmSubsystem armSubsystem, double Arm2Position, double Arm3Position ){
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

    }

    @Override
    public boolean isFinished() {
        boolean Arm2InPosition = false , Arm3InPosition = false;

        if (Math.abs(armSubsystem.CurrentArm2AltPos() - DesiredArm2Position) <= .01){Arm2InPosition = true;}
        if (Math.abs(armSubsystem.CurrentArm3AltPos() - DesiredArm3Position) <= .01){Arm3InPosition = true;}
        

        if(Arm2InPosition && Arm3InPosition){
            return true;
        }else{
            return false;
        }
    }


}

