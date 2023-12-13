  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

   public static final double FlyWheelChainGearRatio = 24.0 / 42.0; // need to change
   public static final int FlyWheelUnitsPerFXRotation = 2048;
            public static final double RPM = FlyWheelChainGearRatio / 60.0 * FlyWheelUnitsPerFXRotation / 10.0;
            public static final double DefaultFlyWheelVelocity = 4500 * RPM; // * RPM

            public static final double PaddlePercentOutput = 0.7;
 
          
   WPI_TalonFX flyWheelMotor; 
   WPI_TalonSRX paddleMotor;

   DigitalInput hallEffect;

public enum ShooterState{ Stopped, SpinningUpFlyWheel, AdvancePaddle, IndexingPaddle};
ShooterState shooterState = ShooterState.Stopped;

  public Shooter() {
   flyWheelMotor = new WPI_TalonFX(Constants.MotorIDs.flyWheelMotorID);
   paddleMotor = new WPI_TalonSRX(Constants.MotorIDs.paddleMotorID);
   
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void manageShooterState() {
   switch(shooterState){
    
    case Stopped:   
    stopFlyWheel();
    stopPaddle();
    break;
   
    case AdvancePaddle:
    startPaddle();
    break;
    
    case SpinningUpFlyWheel: 
    startFlywheel();
    if (isFlyWheelUpToSpeed()){
      shooterState = ShooterState.AdvancePaddle;
    }
    break;
    
    case IndexingPaddle:
    startPaddle();
    
    break;
    
   }
  }
  
  public void startFlywheel() {
    flyWheelMotor.set(TalonFXControlMode.Velocity, DefaultFlyWheelVelocity);
     
    
   }

public void startPaddle(){
  paddleMotor.set(TalonSRXControlMode.PercentOutput, PaddlePercentOutput);

  //Fix value of paddle motor
}

public void stopFlyWheel(){
  flyWheelMotor.set(TalonFXControlMode.Velocity, 0);

}
  public void stopPaddle(){  
   paddleMotor.set(TalonSRXControlMode.PercentOutput, 0);
   
  }
public double getFlyWheelMotorVelocity(){
  return flyWheelMotor.getSelectedSensorVelocity();

}

  public boolean isFlyWheelUpToSpeed(){
    return (getFlyWheelMotorVelocity() *.95 >=DefaultFlyWheelVelocity);
  
  }
  
  public boolean isIndexed() {
    return !hallEffect.get();
  }
  
}
