package frc.robot.commands.Auto1;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.ShootBallsAllStop;
import frc.robot.commands.CommandGroups.ShootSeparatefromFeed;
import frc.robot.commands.Drivetrain.DriveAuto;
import frc.robot.commands.Drivetrain.stopDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.DrivetrainSub.Drivetrain;
import frc.robot.subsystems.Shooter.Shooter;

public class PymblePride {

    public PymblePride( ) 
    {
        
    }


    public static Command getAutonomousCommand(Intake intake,
    Drivetrain drivetrain,
    Feeder feeder,
    Shooter shooter,
    Spindexer spindexer) {

        return new SequentialCommandGroup(
            new ShootSeparatefromFeed(shooter, spindexer, feeder, intake).withTimeout(8),
         new ShootBallsAllStop(shooter, spindexer, feeder, intake).withTimeout(1),
    new DriveAuto(drivetrain).withTimeout(2), new stopDrive(drivetrain));     }
    
}
