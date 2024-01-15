package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.RowFilter.ComparisonType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import frc.robot.Constants.ArmContants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax Arm1SparkMax = new CANSparkMax(ArmContants.Arm1CANID, MotorType.kBrushless);
    private CANSparkMax Arm2SparkMax = new CANSparkMax(ArmContants.Arm2CANID, MotorType.kBrushless);
    private CANSparkMax Arm3SparkMax = new CANSparkMax(ArmContants.Arm3CANID, MotorType.kBrushless);

    private SparkMaxPIDController Arm1PID = Arm1SparkMax.getPIDController();
    private SparkMaxPIDController Arm2PID = Arm2SparkMax.getPIDController();
    private SparkMaxPIDController Arm3PID = Arm3SparkMax.getPIDController();

    public ArmSubsystem() {

        Arm1SparkMax.restoreFactoryDefaults();
        Arm1SparkMax.setIdleMode(IdleMode.kBrake);
        Arm1SparkMax.setSmartCurrentLimit(40);
        Arm1SparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Arm1SparkMax.setSoftLimit(SoftLimitDirection.kForward, 40);

        Arm2SparkMax.restoreFactoryDefaults();
        Arm2SparkMax.setIdleMode(IdleMode.kBrake);
        Arm2SparkMax.setSmartCurrentLimit(40);
        Arm2SparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Arm2SparkMax.setSoftLimit(SoftLimitDirection.kForward, 40);

        Arm3SparkMax.restoreFactoryDefaults();
        Arm3SparkMax.setIdleMode(IdleMode.kBrake);
        Arm3SparkMax.setSmartCurrentLimit(40);
        Arm3SparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
        Arm3SparkMax.setSoftLimit(SoftLimitDirection.kForward, 40);

        // PID Controllers information
        Arm1PID.setP(0.00035);
        // Arm1PID.setI(0);
        // Arm1PID.setD(0);
        Arm1PID.setOutputRange(-1, 1);
        Arm1PID.setSmartMotionMaxAccel(4000, 0);
        Arm1PID.setSmartMotionMaxVelocity(6000, 0);

        // PID Controllers information
        Arm2PID.setP(0.0005);
        // Arm2PID.setI(0);
        // Arm2PID.setD(0);
        Arm2PID.setOutputRange(-1, 1);
        Arm2PID.setSmartMotionMaxAccel(4000, 0);
        Arm2PID.setSmartMotionMaxVelocity(6000, 0);

        // PID Controllers information
        Arm3PID.setP(0.00015);
        // Arm3PID.setI(0);
        // Arm3PID.setD(0);
        Arm3PID.setOutputRange(-1, 1);
        Arm3PID.setSmartMotionMaxAccel(15000, 0);
        Arm3PID.setSmartMotionMaxVelocity(15000, 0);

    }

    public void setArm1Position(double DesiredArmPosition) {
       /*  if (CurrentArm1Pos() > DesiredArmPosition) { // we are attempting to move the arm backward

            if (DesiredArmPosition <= ArmContants.Arm1bottomlimit) {
                Arm1PID.setReference(ArmContants.Arm1bottomlimit, CANSparkMax.ControlType.kSmartMotion);
            } else if (DesiredArmPosition >= ArmContants.Arm1toplimit) {
                Arm1PID.setReference(ArmContants.Arm1toplimit, CANSparkMax.ControlType.kSmartMotion);
            } else {
                Arm1PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
            }

        } else if (CurrentArm1Pos() < DesiredArmPosition) { // we are attempting to move the arm forward
            if (isArm1SafeToMoveForward()) {
                // move Arm 1 to the desired position
                if (DesiredArmPosition <= ArmContants.Arm1bottomlimit) {
                    Arm1PID.setReference(ArmContants.Arm1bottomlimit, CANSparkMax.ControlType.kSmartMotion);
                } else if (DesiredArmPosition >= ArmContants.Arm1toplimit) {
                    Arm1PID.setReference(ArmContants.Arm1toplimit, CANSparkMax.ControlType.kSmartMotion);
                } else {
                    Arm1PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
                }
            }

        } else {
            // the arm is in the current DesiredPosition so do nothing
        } */
        Arm1PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setArm2Position(double DesiredArmPosition) {

       /*  if(CurrentArm2Pos() < DesiredArmPosition) { //moves arm out
            if (DesiredArmPosition >= ArmContants.Arm2toplimit) {
                Arm2PID.setReference(ArmContants.Arm2toplimit, CANSparkMax.ControlType.kSmartMotion);
            } else {
            Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
            } else if (CurrentArm2Pos() > DesiredArmPosition) {
                if (CurrentArm3Pos() > ArmContants.Arm1SafeThreshold) {
                    if (DesiredArmPosition < ArmContants.Arm2SafeThreshold) {
                        Arm2PID.setReference(ArmContants.Arm2SafeThreshold, CANSparkMax.ControlType.kSmartMotion);

        } */






       /* if (CurrentArm2Pos() < DesiredArmPosition) { // Moving Arm 2 out to either the top limit or the DesiredPosition
                                                     // (whichever is smaller)
            if (DesiredArmPosition >= ArmContants.Arm2toplimit) {
                Arm2PID.setReference(ArmContants.Arm2toplimit, CANSparkMax.ControlType.kSmartMotion);
            } else {
                Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
            }
        } else if (CurrentArm2Pos() > DesiredArmPosition) { // Moving Arm 2 In - check whether arm 1 is in a safe
                                                            // position
            if (CurrentArm1Pos() > ArmContants.Arm1SafeThreshold) {
                if (DesiredArmPosition < ArmContants.Arm2SafeThreshold) {
                    Arm2PID.setReference(ArmContants.Arm2SafeThreshold, CANSparkMax.ControlType.kSmartMotion);
                } else {
                    Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
                }
            } else {
                if (DesiredArmPosition <= ArmContants.Arm2bottomlimit) {
                    Arm2PID.setReference(ArmContants.Arm2bottomlimit, CANSparkMax.ControlType.kSmartMotion);
                } else {
                    Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
                }
            }
        }*/
        Arm2PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
    }

    public void setArm3Position(double DesiredArmPosition) {
        // if arm 2 is out past the threshold then it's safe to move arm3
       /* if (CurrentArm3Pos() <= DesiredArmPosition) {// we are moving the arm out
            if (CurrentArm2Pos() >= ArmContants.Arm2SafeThreshold) {
                if (DesiredArmPosition >= ArmContants.Arm3toplimit) {
                    Arm3PID.setReference(ArmContants.Arm3toplimit, CANSparkMax.ControlType.kSmartMotion);
                } else {
                    Arm3PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
                }
            }

        } else if (CurrentArm3Pos() >= DesiredArmPosition) { // we are moving the are in
            if (DesiredArmPosition <= ArmContants.Arm3bottomlimit) {
                Arm3PID.setReference(ArmContants.Arm3bottomlimit, CANSparkMax.ControlType.kSmartMotion);
            } else {
                Arm3PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);
            }
        }*/
        Arm3PID.setReference(DesiredArmPosition, CANSparkMax.ControlType.kSmartMotion);


    }

    /*
     * public void IncrementArmPos(double IncrementalPos) {
     * double CurrentPos = Arm1SparkMax.getEncoder().getPosition();
     * Arm1PID.setReference(CurrentPos + IncrementalPos,
     * CANSparkMax.ControlType.kSmartMotion);
     * }
     */

    public double CurrentArm1Pos() {
        return Arm1SparkMax.getEncoder().getPosition();
    }

    public double CurrentArm2Pos() {
        return Arm2SparkMax.getEncoder().getPosition();
    }

    public double CurrentArm3Pos() {
        return Arm3SparkMax.getEncoder().getPosition();
    }

    // This function determines if it is safe to move Arm 1
    // It is safe to move Arm 1 if Arm 2 is above the Arm2SafeThreshold
    public boolean isArm1SafeToMoveForward() {

        if (CurrentArm2Pos() >= ArmContants.Arm2SafeThreshold) {
            return true;
        } else {
            return false;
        }
    }
}
