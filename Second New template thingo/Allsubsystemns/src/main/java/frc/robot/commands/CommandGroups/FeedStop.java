// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Feeder.FeederStop;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Spindexer.SpindexerStop;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Shooter;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedStop extends ParallelCommandGroup {
  /** Creates a new FeedStop. */
  public FeedStop(Feeder feeder, Spindexer spindexer, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FeederStop(feeder), new SpindexerStop(spindexer), new ShooterStop(shooter));
  }
}
