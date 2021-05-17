// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto1;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub.Drivetrain;

public class MoveRobot extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final double dist;

  /** Creates a new MoveRobot. */
  public MoveRobot(Drivetrain drivetrain, double distance) {
    m_drivetrain = drivetrain;
    dist =  distance;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.zeroSensors();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.move(dist);
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.stopDrivetrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_drivetrain.hasreachedTarget();  //m_drivetrain.reachedTarget();
    
  }
}
