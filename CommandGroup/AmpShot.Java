/*
rotate intake arm to parallel to the amp

while sensor detects a ring 
    run the wheels till ring is placed

reset arm position
*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class AmpShot extends Command {

    private ArmSubsystem armSubsystem;
    private double DesiredArm1Position;   

    public AmpShot(ArmSubsystem armSubsystem, double Arm1Position){
        this.DesiredArm1Position = Arm1Position; 

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
       armSubsystem.setArm1Position(0); 

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
