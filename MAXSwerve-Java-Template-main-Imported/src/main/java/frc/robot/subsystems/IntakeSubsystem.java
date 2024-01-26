package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    //private final CANSparkMax intakeToggleMotor = new CANSparkMax(IntakeConstants.IntakeTogglePort, MotorType.kBrushless);
    //private final CANSparkMax intakeLeftMotor = new CANSparkMax(IntakeConstants.IntakeLeftPort, MotorType.kBrushless);
    //private final CANSparkMax intakeRightMotor = new CANSparkMax(IntakeConstants.IntakeRightPort, MotorType.kBrushless);

   // private SparkMaxPIDController  intakeTogglePID = intakeToggleMotor.getPIDController();

   private final CANSparkMax Intake = new CANSparkMax(IntakeConstants.Intake, MotorType.kBrushless);

    public IntakeSubsystem() {

        Intake.setIdleMode(IdleMode.kBrake);

        //setup the defaults for the intake motor controllers
        //intakeToggleMotor.setIdleMode(IdleMode.kBrake);
        //intakeLeftMotor.setIdleMode(IdleMode.kBrake);
        //intakeRightMotor.setIdleMode(IdleMode.kBrake);
        //intakeRightMotor.setInverted(true);
        //intakeTogglePID.setP(0.05);
    }

    public void runIntake(double speed) {
        //intakeLeftMotor.set(speed);
        //intakeRightMotor.set(speed);
        Intake.set(speed);
    }

    public void intakeTogglePosition(double togglePosition) {
       // if(togglePosition == 0) {
            // intakeTogglePID.setReference(IntakeConstants.IntakeClose, CANSparkMax.ControlType.kPosition);
       // }
       // else if(togglePosition == 1) {
          //  intakeTogglePID.setReference(IntakeConstants.IntakeOpen, CANSparkMax.ControlType.kPosition);
       // }   
    }

    public boolean isOpen() {
        //if(intakeToggleMotor.getEncoder().getPosition() <= IntakeConstants.IntakeOpen - 2 || 
                //intakeToggleMotor.getEncoder().getPosition() >= IntakeConstants.IntakeOpen + 2 ) {
            //return false;
        //}
        //else {
            return true;
        //}
        
    }
 
    public double CurrentIntakePosition() {
        return Intake.getEncoder().getPosition();

    }
}