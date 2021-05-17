// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {
    robotInit();
  }

  WPI_TalonFX shooter_left = new WPI_TalonFX(1); // needs to be imverted
  WPI_TalonFX shooter_right = new WPI_TalonFX(2);

  DoubleSolenoid intake_pneu = new DoubleSolenoid(0,1);



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
/** 
  public void shootershoot(){
    shooter_left.set(1);
    shooter_right.set(1);
  }

  public void shooterstop(){
    shooter_left.set(0);
    shooter_right.set(0);
  }*/

  public void shoot40(){
    shooter_left.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kF, Constants.kTimeoutMs);
    shooter_left.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kP, Constants.kTimeoutMs);
    shooter_left.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kI, Constants.kTimeoutMs);
    shooter_left.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kD, Constants.kTimeoutMs);
  
    double targetVelocity_UnitsPer100ms = 2552 * 2048 / 600;
        
    shooter_left.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }
  
    public void shoot50(){
      shooter_left.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kF, Constants.kTimeoutMs);
      shooter_left.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kP, Constants.kTimeoutMs);
      shooter_left.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kI, Constants.kTimeoutMs);
      shooter_left.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kD, Constants.kTimeoutMs);
    
      double targetVelocity_UnitsPer100ms = 3188 * 2048 / 600;
          
      shooter_left.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }
     public void shoot60(){
      shooter_left.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kF, Constants.kTimeoutMs);
      shooter_left.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kP, Constants.kTimeoutMs);
      shooter_left.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kI, Constants.kTimeoutMs);
      shooter_left.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kD, Constants.kTimeoutMs);
    
      double targetVelocity_UnitsPer100ms = 3825 * 2048 / 600;
          
      shooter_left.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
      }
      
    public void shoot80(){
      shooter_left.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kF, Constants.kTimeoutMs);
      shooter_left.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kP, Constants.kTimeoutMs);
      shooter_left.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kI, Constants.kTimeoutMs);
      shooter_left.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kD, Constants.kTimeoutMs);
    
      double targetVelocity_UnitsPer100ms = 5100 * 2048 / 600;
          
      shooter_left.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  
    }
    
  
  
    public void stopShooter(){
      shooter_left.set(ControlMode.PercentOutput,0.0);
    }
  
    public void robotInit(){
      shooter_left.configFactoryDefault();
      shooter_right.configFactoryDefault();
  
      shooter_right.follow(shooter_left);
  
      
      shooter_right.setInverted(false);
      shooter_left.setInverted(true);
      
       
      shooter_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
                                            Constants.kTimeoutMs);
  
          
      shooter_left.setSensorPhase(true);
  
      shooter_left.configNominalOutputForward(0, Constants.kTimeoutMs);
      shooter_left.configNominalOutputReverse(0, Constants.kTimeoutMs);
      shooter_left.configPeakOutputForward(1, Constants.kTimeoutMs);
      shooter_left.configPeakOutputReverse(-1, Constants.kTimeoutMs);

  
    }


}
