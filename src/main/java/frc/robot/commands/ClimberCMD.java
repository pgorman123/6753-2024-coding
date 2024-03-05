package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCMD extends Command {

    private ClimberSubsystem climberSubsystem;   
  
    private double DesiredClimberLeftPosition;
    private double DesiredClimberRightPosition;

    public ClimberCMD(ClimberSubsystem climberSubsystem, double ClimberLeftPosition, double ClimberRightPosition ){
        this.DesiredClimberLeftPosition = ClimberLeftPosition;
        //this.DesiredClimberRightPosition = ClimberRightPosition;
    
    
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }


    /*public ClimberCMD(ClimberSubsystem theClimberSystem, int i, int j, Translation2d translation2d, Rotation2d rotation2d,
            Translation2d translation2d2, Rotation2d rotation2d2, ClimberCMD climberCMD,
            IntakeRunWheelsCMD intakeRunWheelsCMD, IntakeRunWheelsCMD intakeRunWheelsCMD2, NewIntakeArmSetCMD mewintakearmSetCMD2,
            Translation2d translation2d3, Rotation2d rotation2d3, TrajectoryConfig config) {
    }*/

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberSubsystem.setClimberLeftPosition(DesiredClimberLeftPosition);
        climberSubsystem.setClimberRightPosition(DesiredClimberRightPosition);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}