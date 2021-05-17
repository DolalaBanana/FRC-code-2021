// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Feeder.FeederStop;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Spindexer.SpindexerStop;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBallsAllStop extends ParallelCommandGroup {
  /** Creates a new ShootBallsAllStop. */
  public ShootBallsAllStop(Shooter shooter, Spindexer spindexer, Feeder feeder, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShooterStop(shooter), new SpindexerStop(spindexer), new FeederStop(feeder), new IntakeIn(intake));
  }
}
