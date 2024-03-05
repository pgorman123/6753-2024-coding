package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewIntakeArmSubsystem;

public class NewIntakeArmSetCMD extends Command {

    private NewIntakeArmSubsystem newintakearmSubsystem;   
  
    private double DesiredIntakeArmPosition;

    public NewIntakeArmSetCMD(NewIntakeArmSubsystem newintakearmSubsystem, double IntakeArmPosition ){
        this.DesiredIntakeArmPosition = IntakeArmPosition;

        this.newintakearmSubsystem = newintakearmSubsystem;
        addRequirements(newintakearmSubsystem);
    }

    public NewIntakeArmSetCMD(NewIntakeArmSubsystem theNewIntakeArmSystem, int i, int j, Translation2d translation2d, Rotation2d rotation2d,
            Translation2d translation2d2, Rotation2d rotation2d2, NewIntakeArmSetCMD newintakearmSetCMD,
            IntakeRunWheelsCMD intakeRunWheelsCMD, IntakeRunWheelsCMD intakeRunWheelsCMD2, NewIntakeArmSetCMD mewintakearmSetCMD2,
            Translation2d translation2d3, Rotation2d rotation2d3, TrajectoryConfig config) {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        newintakearmSubsystem.setIntakeArmPosition(DesiredIntakeArmPosition);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}