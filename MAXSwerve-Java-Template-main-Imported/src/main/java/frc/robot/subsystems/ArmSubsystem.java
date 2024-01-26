package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.RowFilter.ComparisonType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import frc.robot.Constants.ArmContants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax Arm2FSparkMax = new CANSparkMax(ArmContants.Arm2FCANID, MotorType.kBrushless);
    private CANSparkMax Arm2SparkMax = new CANSparkMax(ArmContants.Arm2CANID, MotorType.kBrushless);
    private CANSparkMax Arm3SparkMax = new CANSparkMax(ArmContants.Arm3CANID, MotorType.kBrushless);

    private SparkMaxPIDController Arm2FPID = Arm2FSparkMax.getPIDController();
    private SparkMaxPIDController Arm2PID = Arm2SparkMax.getPIDController();
    private SparkMaxPIDController Arm3PID = Arm3SparkMax.getPIDController();

    private static final SparkMaxAlternateEncoder.Type Arm2EncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int Arm2CPR = 8192;
    private RelativeEncoder Arm2alternateEncoder;

    private static final SparkMaxAlternateEncoder.Type Arm3EncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int Arm3CPR = 8192;
    private RelativeEncoder Arm3alternateEncoder;


    public ArmSubsystem() {

        Arm2FSparkMax.restoreFactoryDefaults();
        Arm2FSparkMax.setIdleMode(IdleMode.kCoast);
        Arm2FSparkMax.setSmartCurrentLimit(40);
        
        Arm2FSparkMax.follow(Arm2SparkMax, true);

        Arm2SparkMax.restoreFactoryDefaults();
        Arm2SparkMax.setIdleMode(IdleMode.kCoast);
        Arm2SparkMax.setSmartCurrentLimit(40);
        Arm2SparkMax.setInverted(true);
       
        Arm3SparkMax.restoreFactoryDefaults();
        Arm3SparkMax.setIdleMode(IdleMode.kBrake);
        Arm3SparkMax.setSmartCurrentLimit(40);
        Arm3SparkMax.setInverted(true);
       
        // PID Controllers information
        Arm2PID.setP(.0012); //0.0025 New value of .0012 after gear changes
        // Arm2PID.setI(0);
        // Arm2PID.setD(0);
        Arm2PID.setOutputRange(-1, 1);
        Arm2PID.setSmartMotionMaxAccel(3000, 0);
        Arm2PID.setSmartMotionMaxVelocity(7000, 0);

        // PID Controllers information 
        Arm3PID.setP(.0008);  //0.0006
        // Arm3PID.setI(0);
         Arm3PID.setD(.01);
        Arm3PID.setOutputRange(-1, 1);
        Arm3PID.setSmartMotionMaxAccel(8000, 0);
        Arm3PID.setSmartMotionMaxVelocity(12000, 0);

        Arm2alternateEncoder = Arm2SparkMax.getAlternateEncoder(Arm2EncodeType, Arm2CPR);
        Arm2PID.setFeedbackDevice(Arm2alternateEncoder);

        Arm3alternateEncoder = Arm3SparkMax.getAlternateEncoder(Arm3EncodeType, Arm3CPR);
        //Arm3alternateEncoder.setInverted(true); // the encoder was moved to the other side of the shaft so it no longer needs to be inverted
        Arm3PID.setFeedbackDevice(Arm3alternateEncoder);

    }

    public void setArm2Position(double DesiredArmPosition) {

        Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setArm3Position(double DesiredArmPosition) {
    
        Arm3PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);


    }


   
    public double CurrentArm2MotorPos() {
        return Arm2SparkMax.getEncoder().getPosition();
    }

    public double CurrentArm3MotorPos() {
        return Arm3SparkMax.getEncoder().getPosition();
    }

    public double CurrentArm2AltPos() {
        return Arm2alternateEncoder.getPosition();
    }

    public double CurrentArm3AltPos() {
        return Arm3alternateEncoder.getPosition();
    }

    public double CurrentArm2MotorAppliedOutput() {
        return Arm2SparkMax.getAppliedOutput();
    }

    public double CurrentArm2FMotorAppliedOutput() {
        return Arm2FSparkMax.getAppliedOutput();
    }

    public double Arm2Velocity() {
        return Arm2SparkMax.getEncoder().getVelocity();
    }

    public double Arm3Velocity() {
        return Arm3SparkMax.getEncoder().getVelocity();
    }

    public double Arm3AppliedOutput() {
        return Arm3SparkMax.getAppliedOutput();
    }

}   
