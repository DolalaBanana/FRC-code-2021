// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */ 
  WPI_VictorSPX spindexer = new WPI_VictorSPX(7);
  Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);


  public Spindexer() {
    
    distanceSensor.setAutomaticMode(true);
    spindexer.setNeutralMode(NeutralMode.Brake);

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void detect_ball(){

    if( distanceSensor.isRangeValid()){
      SmartDashboard.putNumber("getRange", distanceSensor.getRange());
    }

    if(distanceSensor.getRange() < 5 && distanceSensor.isRangeValid()){
  
      spindexer.set(.6); // was -0.6
      

    }else{
      spindexer.set(0);

    }



  }

    public void spin_spindexer(){
      spindexer.set(1);
    }

    public void stop_spindexer(){
      spindexer.set(0);
    }


}
