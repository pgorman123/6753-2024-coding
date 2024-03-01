package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.ShooterConstants;

public class ShooterTipSubsystem extends SubsystemBase {

   
    private CANSparkMax ShooterTipSparkMax = new CANSparkMax(ShooterConstants.ShooterTip, MotorType.kBrushless);

    private SparkPIDController ShooterTipPID = ShooterTipSparkMax.getPIDController();
    

    private static final SparkMaxAlternateEncoder.Type ShooterTipEncodeType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int ShooterTipCPR = 8192;
    private RelativeEncoder ShooterTipalternateEncoder;


    public ShooterTipSubsystem() {


        ShooterTipSparkMax.restoreFactoryDefaults();
        ShooterTipSparkMax.setIdleMode(IdleMode.kBrake);
        ShooterTipSparkMax.setSmartCurrentLimit(40);
        ShooterTipSparkMax.setInverted(true);
       
        

        // PID Controllers information 
        ShooterTipPID.setP(.0008);  //0.0006. This is a comment from 2023
        // ShooterTipPID.setI(0);
        ShooterTipPID.setD(.01);
        ShooterTipPID.setOutputRange(-0.5, 0.5); //was -1 and 1 (2/24/24)
        ShooterTipPID.setSmartMotionMaxAccel(4000, 0); //was 8000 (2/24/24)
        ShooterTipPID.setSmartMotionMaxVelocity(6000, 0); //was 12000 (2/24/24)


        ShooterTipalternateEncoder = ShooterTipSparkMax.getAlternateEncoder(ShooterTipEncodeType, ShooterTipCPR);
        //ShooterTipalternateEncoder.setInverted(true); // the encoder was moved to the other side of the shaft so it no longer needs to be inverted
        ShooterTipPID.setFeedbackDevice(ShooterTipalternateEncoder);

    }

    public void setShooterArmPosition(double DesiredTipPosition) {

        
    }

    public void setShooterTipPosition(double DesiredTipPosition) {
    
        ShooterTipPID.setReference(DesiredTipPosition, CANSparkMax.ControlType.kSmartMotion);


    }


    public double CurrentShooterTipMotorPos() {
        return ShooterTipSparkMax.getEncoder().getPosition();
    }

    public double CurrentShooterTipAltPos() {
        return ShooterTipalternateEncoder.getPosition();
    }

    public double ShooterTipVelocity() {
        return ShooterTipSparkMax.getEncoder().getVelocity();
    }

    public double ShooterTipAppliedOutput() {
        return ShooterTipSparkMax.getAppliedOutput();
    }

}   
