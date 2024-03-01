package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{


   private final CANSparkMax Intake = new CANSparkMax(IntakeConstants.Intake, MotorType.kBrushless);

    public IntakeSubsystem() {

        Intake.setIdleMode(IdleMode.kBrake);
    }

    public void runIntake(double speed) {
       
        Intake.set(speed);
    }

    public void intakeTogglePosition(double togglePosition) {
    }

    public boolean isOpen() {
       
            return true;
    }
 
    public double CurrentIntakePosition() {
        return Intake.getEncoder().getPosition();
    }
}