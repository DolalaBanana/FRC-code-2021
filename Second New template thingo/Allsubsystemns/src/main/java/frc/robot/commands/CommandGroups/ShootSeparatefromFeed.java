// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.Shooter80;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSeparatefromFeed extends SequentialCommandGroup {
  /** Creates a new ShootSeparatefromFeed. */
  public ShootSeparatefromFeed(Shooter m_shooter, Spindexer m_spindexer, Feeder m_feeder, Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Shooter80(m_shooter).withTimeout(2), new ShootingtheBalls(m_shooter, m_spindexer, m_feeder, m_intake));
  }
}
