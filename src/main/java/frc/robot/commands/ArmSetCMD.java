package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetCMD extends Command {

    private ArmSubsystem armSubsystem;
    private double DesiredArm1Position;   
    private double DesiredArm2Position;
    private double DesiredArm3Position;

    public ArmSetCMD(ArmSubsystem armSubsystem, double Arm1Position, double Arm2Position, double Arm3Position ){
        this.DesiredArm1Position = Arm1Position; 
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
        armSubsystem.setArm1Position(DesiredArm1Position);
        armSubsystem.setArm2Position(DesiredArm2Position);
        armSubsystem.setArm3Position(DesiredArm3Position);
    }

    @Override
    public void end(boolean interrupted) {
       // armSubsystem.setArm1Position(0); 
        //armSubsystem.setArm2Position(0);
        //armSubsystem.setArm3Position(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}

