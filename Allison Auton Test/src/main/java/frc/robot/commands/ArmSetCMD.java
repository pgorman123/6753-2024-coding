package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetCMD extends Command {

    private ArmSubsystem armSubsystem;   
    private double DesiredArm2Position;
    private double DesiredArm3Position;

    public ArmSetCMD(ArmSubsystem armSubsystem, double Arm2Position, double Arm3Position ){
        this.DesiredArm2Position = Arm2Position;
        this.DesiredArm3Position = Arm3Position;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    public ArmSetCMD(ArmSubsystem theArmSystem, int i, int j, Translation2d translation2d, Rotation2d rotation2d,
            Translation2d translation2d2, Rotation2d rotation2d2, ArmSetCMD armSetCMD,
            IntakeRunWheelsCMD intakeRunWheelsCMD, IntakeRunWheelsCMD intakeRunWheelsCMD2, ArmSetCMD armSetCMD2,
            Translation2d translation2d3, Rotation2d rotation2d3, TrajectoryConfig config) {
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
        return false;
    }


}

