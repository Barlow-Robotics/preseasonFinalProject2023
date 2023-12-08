// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
   WPI_TalonFX flyWheelMotor; 
   WPI_TalonSRX paddleMotor;

public enum ShooterState{ Stopped, SpinningUpFlyWheel, UnIndexingPaddle, FinishShooting, IndexingPaddle};

  public Shooter() {
   flyWheelMotor = new WPI_TalonFX(Constants.MotorIDs.flyWheelMotorID);
   paddleMotor = new WPI_TalonSRX(Constants.MotorIDs.paddleMotorID);
   
  }
  public boolean StartFlywheel() {
    flyWheelMotor.set(TalonFXControlMode.Velocity, 1);
    return true; 
   }
  public void ShootNDiscs(){
    //paddleMotor.set(TalonSRXControlMode.Velocity, double value);

  } 

public boolean StartShooter(){
  paddleMotor.set(TalonSRXControlMode.Velocity, 1);
  return true;
  //Fix value of paddle motor
}

public boolean StopFlyWheel(){
  flyWheelMotor.set(TalonFXControlMode.Velocity, 0);
  return true;
}
  public boolean StopShooter(){  
   paddleMotor.set(TalonSRXControlMode.Velocity, 0);
   return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
