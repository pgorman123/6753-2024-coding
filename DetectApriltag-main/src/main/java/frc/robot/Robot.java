// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Delayed;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  PhotonCamera mPVCamera = new PhotonCamera("Limelight1");
  boolean mHasTarget;
  PhotonPipelineResult mResult;
  PhotonTrackedTarget mTarget;
  double mX;

  private Joystick js1 = new Joystick(0);
  private DigitalOutput m_LeftLight = new DigitalOutput(2);
  private DigitalOutput m_RightLight = new DigitalOutput(4);

  // PhotonCamera camera = new PhotonCamera("WEB_CAM");
  // PhotonPipelineResult result;
  // boolean hasTarget;
  // List<PhotonTrackedTarget> targets;

  // Define target
  // PhotonTrackedTarget target;
  // Transform3d bestCameraToTarget;

  // Joystick
  Joystick js = new Joystick(0);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void teleopInit() {}

  /* This function is called periodically during autonomous. */
  @Override
  public void teleopPeriodic() {
    m_LeftLight.set(!true);
    m_RightLight.set(!false);

    // if( js1.getRawButton(1) ) LeftLight.set(false); 

    // SmartDashboard.putBoolean("Button", js1.getRawButton(1));
    SmartDashboard.putBoolean("LeftLight", !m_LeftLight.get());
    SmartDashboard.putBoolean("RightLight", !m_RightLight.get());
  }

  @Override
  public void autonomousInit() {
    m_LeftLight.set(!true);
    m_RightLight.set(!true);
    mX = 0;
  }

  @Override
  public void autonomousPeriodic() {
    // TODO_01
    /*
     *  宣告並獲取 Apriltag 要用的資料 
     */
    
    // Example Code
    //  result = ???;
    //  hasTarget = ???;
    mResult = mPVCamera.getLatestResult();
    mHasTarget = mResult.hasTargets();

    // TODO_02
    /*
     * PS 0. Task 順序不一定是對的喔，可能要先 2. 再 1. 也可能先 1. 再 2. 也可能要 1. 2. 同時
     * Task 1. 輸出從鏡頭得到的資料
     *  PS 1. 輸出資料所使用的函式
     *    PS 1-1. 輸出鏡頭中是否有 Apriltag : SmartDashboard.putBoolean( ???, ??? );
     *    PS 1-2. 輸出鏡頭跟 Apriltag 的距離 : SmartDashboard.putNumber( ???, ??? );
     * Task 2. 思考可能會有問題的地方
     *  PS 2. 提供一個資訊：如果鏡頭中沒有 Apriltag 但是你還跟他要資料的話整個程式都會掛掉，想一下要怎麼處理
     */

    if( mHasTarget ){
      mTarget = mResult.getBestTarget();
      Transform3d mCameraOutput = getCameratoTarget();
      mX = mCameraOutput.getY();
    }

    if( mX < -0.2 ){
      m_LeftLight.set(!true);
      m_RightLight.set(!false);
    }else if( mX > 0.2 ){
      m_LeftLight.set(!false);
      m_RightLight.set(!true);
    }else{ 
      m_LeftLight.set(!false);
      m_RightLight.set(!false);
    }

    SmartDashboard.putBoolean("HasTarget", mHasTarget);
    SmartDashboard.putNumber("X", mX);
    SmartDashboard.putBoolean("LeftLight", !m_LeftLight.get());
    SmartDashboard.putBoolean("RightLight", !m_RightLight.get());
  }

  /**
   * Get the transform that maps camera ( X = forward, Y = left, Z = up ) to apriltag
   **/
  public Transform3d getCameratoTarget(){
    return mHasTarget == true ? mTarget.getBestCameraToTarget() : new Transform3d( new Translation3d(0,0,0), new Rotation3d(0,0,0) );
  }
}