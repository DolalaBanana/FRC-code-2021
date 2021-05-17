// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;
import frc.robot.commands.Spindexer.SpindexerSpin;
import frc.robot.commands.Feeder.FeederUp;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedBallsForShooter extends ParallelCommandGroup {
  /** Creates a new FeedBallsForShooter. */
  public FeedBallsForShooter(Feeder feeder, Spindexer spindexer) {
  
    addCommands(new FeederUp(feeder), new SpindexerSpin(spindexer));
  }
}
