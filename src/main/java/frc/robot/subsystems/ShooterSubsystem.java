package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ArmContants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{


   private final CANSparkMax ShooterF = new CANSparkMax(ShooterConstants.ShooterF, MotorType.kBrushless);
   private CANSparkMax Shooter = new CANSparkMax(ShooterConstants.Shooter, MotorType.kBrushless);

    public ShooterSubsystem() {

        //not sure if follower code will work
        ShooterF.setIdleMode(IdleMode.kBrake);
        ShooterF.follow(Shooter, true);

        Shooter.setIdleMode(IdleMode.kBrake);


    }

    public void runShooter(double speed) {
       
        Shooter.set(speed);
    }

    public void ShooterTogglePosition(double togglePosition) {
    }

    public boolean isOpen() {
       
            return true;
    }
 
    public double CurrentShooterPosition() {
        return Shooter.getEncoder().getPosition();
    }
    public double getShooterVelocity () {
    return Shooter.getEncoder().getVelocity();
}
    
}