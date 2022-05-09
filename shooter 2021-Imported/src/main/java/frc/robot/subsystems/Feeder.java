// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class Feeder extends SubsystemBase {
  WPI_TalonFX Feeder = new WPI_TalonFX(6);
 
  
  /** Creates a new Feeder. */
  public Feeder() {}

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
      
  public void FeedBalls(){
    Feeder.set(ControlMode.PercentOutput,-0.5);
  }
  public void StopFeed(){
    Feeder.set(ControlMode.PercentOutput,0.0);
  }
}
