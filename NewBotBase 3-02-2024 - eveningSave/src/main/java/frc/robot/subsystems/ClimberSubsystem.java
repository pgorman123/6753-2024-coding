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
import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkBase;

public class ClimberSubsystem extends SubsystemBase {


    private CANSparkMax ClimberLeftSparkMax = new CANSparkMax(ClimberConstants.ClimberLeftCANID, MotorType.kBrushless);
    private CANSparkMax ClimberRightSparkMax = new CANSparkMax(ClimberConstants.ClimberRightCANID, MotorType.kBrushless);

  
    private SparkPIDController ClimberLeftPID = ClimberLeftSparkMax.getPIDController();
    private SparkPIDController ClimberRightPID = ClimberRightSparkMax.getPIDController();


    private static final SparkMaxAlternateEncoder.Type ClimberLeftEncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int ClimberLeftCPR = 8192;
    private RelativeEncoder ClimberLeftalternateEncoder;

    private static final SparkMaxAlternateEncoder.Type ClimberRightEncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int ClimberRightCPR = 8192;
    private RelativeEncoder ClimberRightalternateEncoder; 


    public ClimberSubsystem() {


        ClimberLeftSparkMax.restoreFactoryDefaults(); 
        ClimberLeftSparkMax.setIdleMode(IdleMode.kBrake);
        ClimberLeftSparkMax.setSmartCurrentLimit(40);
        ClimberLeftSparkMax.setInverted(false);

        ClimberRightSparkMax.restoreFactoryDefaults();
        ClimberRightSparkMax.setIdleMode(IdleMode.kBrake); 
        ClimberRightSparkMax.setSmartCurrentLimit(40);
        ClimberRightSparkMax.setInverted(false);
       

        // PID Controllers information 
        ClimberLeftPID.setP(.0008);  //0.0006
        //IntakeArmPID.setI(.0001);
        ClimberLeftPID.setD(.01);
        ClimberLeftPID.setOutputRange(-1, 1);
        ClimberLeftPID.setSmartMotionMaxAccel(8000, 0);
        ClimberLeftPID.setSmartMotionMaxVelocity(12000, 0);

         // PID Controllers information 
        ClimberRightPID.setP(.0008);  //0.0006
        //IntakeArmPID.setI(.0001);
        ClimberRightPID.setD(.01);
        ClimberRightPID.setOutputRange(-1, 1);
        ClimberRightPID.setSmartMotionMaxAccel(8000, 0);
        ClimberRightPID.setSmartMotionMaxVelocity(12000, 0); 




        ClimberLeftalternateEncoder = ClimberLeftSparkMax.getAlternateEncoder(ClimberLeftEncodeType, ClimberLeftCPR);
        //IntakeArmalternateEncoder.setInverted(true); // the encoder was moved to the other side of the shaft so it no longer needs to be inverted
        ClimberLeftPID.setFeedbackDevice(ClimberLeftalternateEncoder);

        ClimberRightalternateEncoder = ClimberRightSparkMax.getAlternateEncoder(ClimberRightEncodeType, ClimberRightCPR);
        //IntakeArmalternateEncoder.setInverted(true); // the encoder was moved to the other side of the shaft so it no longer needs to be inverted
        ClimberRightPID.setFeedbackDevice(ClimberRightalternateEncoder);

    

    }

   

    
    public void setClimberLeftPosition(double DesiredClimberLeftPosition) {
    
        ClimberLeftPID.setReference(DesiredClimberLeftPosition, CANSparkMax.ControlType.kSmartMotion);


    }

    public void setClimberRightPosition(double DesiredClimberRightPosition) {
    
        ClimberRightPID.setReference(DesiredClimberRightPosition, CANSparkMax.ControlType.kSmartMotion);


    }


   
    public double CurrentClimberLeftMotorPos() {
        return ClimberLeftSparkMax.getEncoder().getPosition();
    }

 public double CurrentClimberRightMotorPos() {
        return ClimberRightSparkMax.getEncoder().getPosition();
    }



    public double CurrentClimberLeftAltPos() {
        return ClimberLeftalternateEncoder.getPosition();
    }
    public double CurrentClimberRightAltPos() {
        return ClimberRightalternateEncoder.getPosition();
    }



    public double ClimberLeftVelocity() {
        return ClimberLeftSparkMax.getEncoder().getVelocity();
    }
     public double ClimberRightVelocity() {
        return ClimberRightSparkMax.getEncoder().getVelocity();
    }


    public double ClimberLeftAppliedOutput() {
        return ClimberLeftSparkMax.getAppliedOutput();
    }
    public double ClimberRightAppliedOutput() {
        return ClimberRightSparkMax.getAppliedOutput();
    }


}   
