// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//falcon for hook
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  WPI_TalonFX climber_up = new WPI_TalonFX(0);
  WPI_TalonFX climber_hook = new WPI_TalonFX(12);

  /** Creates a new Climber. */
  public Climber() {
    climber_up.setSelectedSensorPosition(0);
    climber_hook.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp(){
   /* if(climber_up.getSelectedSensorPosition()<74400){
      climber_up.set(.1);
    }
    else{
      climber_up.set(0);

    }*/
    climber_up.set(.2);

  }

  public void climberDown(){
   /* if(climber_up.getSelectedSensorPosition()>100){
      climber_up.set(-.1);
    }
    else{
      climber_up.set(0);

    }*/
    climber_up.set(-.2);

  }

  public void climberStop(){
    climber_up.set(0);
  }

  public void Spoolin(){
    climber_hook.set(1);
  }

  public void Spoolout(){
    climber_hook.set(-1);
  }

  public void Spoolstop(){
    climber_hook.set(0);
  }



}
