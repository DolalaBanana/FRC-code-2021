// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto1;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub.Drivetrain;
import jdk.jshell.execution.DirectExecutionControl;

public class TurnRobot extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final double angle;
  /** Creates a new TurnRobot. */
  public TurnRobot( Drivetrain drivetrain, double ang) {
    m_drivetrain = drivetrain;
    angle = ang;
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


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
