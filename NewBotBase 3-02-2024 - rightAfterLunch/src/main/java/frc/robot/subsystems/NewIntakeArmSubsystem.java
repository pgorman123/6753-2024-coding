package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.RowFilter.ComparisonType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import frc.robot.Constants.ArmContants;
import com.revrobotics.CANSparkBase;

public class NewIntakeArmSubsystem extends SubsystemBase {


    private CANSparkMax IntakeArmSparkMax = new CANSparkMax(ArmContants.IntakeArmCANID, MotorType.kBrushless);
  
    private SparkPIDController IntakeArmPID = IntakeArmSparkMax.getPIDController();


    private static final SparkMaxAlternateEncoder.Type IntakeArmEncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int IntakeArmCPR = 8192;
    private RelativeEncoder IntakeArmalternateEncoder;


    public NewIntakeArmSubsystem() {

       
        IntakeArmSparkMax.restoreFactoryDefaults();
        IntakeArmSparkMax.setIdleMode(IdleMode.kBrake);
        IntakeArmSparkMax.setSmartCurrentLimit(40);
        IntakeArmSparkMax.setInverted(true);
       

        // PID Controllers information 
        IntakeArmPID.setP(.0008);  //0.0006
        //IntakeArmPID.setI(.0001);
         IntakeArmPID.setD(.01);
        IntakeArmPID.setOutputRange(-1, 1);
        IntakeArmPID.setSmartMotionMaxAccel(8000, 0);
        IntakeArmPID.setSmartMotionMaxVelocity(12000, 0);

        

        IntakeArmalternateEncoder = IntakeArmSparkMax.getAlternateEncoder(IntakeArmEncodeType, IntakeArmCPR);
        //IntakeArmalternateEncoder.setInverted(true); // the encoder was moved to the other side of the shaft so it no longer needs to be inverted
        IntakeArmPID.setFeedbackDevice(IntakeArmalternateEncoder);

    }

   

    public void setIntakeArmPosition(double DesiredIntakeArmPosition) {
    
        IntakeArmPID.setReference(DesiredIntakeArmPosition, CANSparkMax.ControlType.kSmartMotion);


    }


   
    public double CurrentIntakeArmMotorPos() {
        return IntakeArmSparkMax.getEncoder().getPosition();
    }


    public double CurrentIntakeArmAltPos() {
        return IntakeArmalternateEncoder.getPosition();
    }


    public double IntakeArmVelocity() {
        return IntakeArmSparkMax.getEncoder().getVelocity();
    }

    public double IntakeArmAppliedOutput() {
        return IntakeArmSparkMax.getAppliedOutput();
    }

}   
