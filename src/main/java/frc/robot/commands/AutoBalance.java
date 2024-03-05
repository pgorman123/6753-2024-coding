package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class AutoBalance extends Command {
    
    private DriveSubsystem driveSubsystem;

    public AutoBalance(DriveSubsystem driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {

        if(driveSubsystem.getRoll() < -10) {
            //drive forward5
            driveSubsystem.drive(.07,0,0,false);
            SmartDashboard.putString("AutoBalance", "forward");
        } else if(driveSubsystem.getRoll() > 10) {
            //drive backwards
            driveSubsystem.drive(-.07,0,0,false);
            SmartDashboard.putString("AutoBalance", "backward");
        } else {
            //stop
            driveSubsystem.setX();
            SmartDashboard.putString("AutoBalance", "stop");
        }

    }

    @Override
    public void end(boolean interrupted) { 
        driveSubsystem.setX();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
