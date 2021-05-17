// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import javax.swing.plaf.basic.BasicButtonUI;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandGroups.FeedStop;
import frc.robot.commands.CommandGroups.IntakeBallsG;
import frc.robot.commands.CommandGroups.IntakeBallsStop;
import frc.robot.commands.CommandGroups.ShootBallsAllStop;
import frc.robot.commands.CommandGroups.ShootSeparatefromFeed;
import frc.robot.commands.CommandGroups.ShootingCloseBalls;
import frc.robot.commands.CommandGroups.ShootingtheBalls;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.HighGear;
import frc.robot.commands.Drivetrain.LowGear;
import frc.robot.commands.Feeder.FeederStop;
import frc.robot.commands.Feeder.FeederUp;
import frc.robot.commands.Auto1.Autonomous1;
import frc.robot.commands.Auto1.MoveRobot;
import frc.robot.commands.Auto1.PymblePride;
import frc.robot.commands.Climber.ClimberDown;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Climber.ClimberUp;
import frc.robot.commands.Climber.SpoolIn;
import frc.robot.commands.Climber.SpoolOut;
import frc.robot.commands.Climber.SpoolStop;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Shooter.Shooter80;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Spindexer.SpindexerSpin;
import frc.robot.commands.Spindexer.SpindexerStop;
import frc.robot.subsystems.DrivetrainSub.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Climber.ClimberUp;
import java.util.function.DoubleSupplier;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Intake m_intake = new Intake();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Spindexer m_spindexer = new Spindexer();
  private final Climber m_climber = new Climber();

  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_driver_joystick = new Joystick(1);
  



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> m_driver_joystick.getRawAxis(1),
    () -> -m_driver_joystick.getRawAxis(4)*0.8, m_drivetrain));
    //m_drivetrain.setDefaultCommand(new ArcadeDrive(0.8, 0.8, m_drivetrain)); 
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this meth
   * od to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    //operator
    final JoystickButton Abutton = new JoystickButton(m_joystick,1);
    final JoystickButton Bbutton = new JoystickButton(m_joystick,2);
    final JoystickButton Xbutton = new JoystickButton(m_joystick,3);
    final JoystickButton Ybutton = new JoystickButton(m_joystick,4);
    final JoystickButton Lbutton = new JoystickButton(m_joystick,5);
    final JoystickButton Rbutton = new JoystickButton(m_joystick,6);

    final JoystickButton Startbutton = new JoystickButton(m_joystick,8);
    final JoystickButton BackBUtton = new JoystickButton(m_joystick,7);


    






//driver
    final JoystickButton elButton = new JoystickButton(m_driver_joystick,5);
    final JoystickButton erButton = new JoystickButton(m_driver_joystick,6);


    



    //Abutton.whenPressed(new ShootingtheBalls(m_spindexer, m_feeder, m_shooter));
    //Abutton.whenReleased(new FeedStop(m_feeder, m_spindexer, m_shooter));

    //Bbutton.whenPressed(new IntakeBalls(m_intake));
    //Bbutton.whenReleased(
    //  new IntakeStop(m_intake);
    //Bbutton.whenReleased(new IntakeIn(m_intake));

//drivetrain
    elButton.whenPressed(new LowGear(m_drivetrain));
    erButton.whenPressed(new HighGear(m_drivetrain));


    //operator
    
    Abutton.whenPressed(new ShootSeparatefromFeed(m_shooter, m_spindexer,m_feeder, m_intake));// this one Brian!!!!
    Abutton.whenReleased(new ShootBallsAllStop(m_shooter, m_spindexer,m_feeder, m_intake));

    Ybutton.whenPressed(new ShootingCloseBalls(m_shooter, m_spindexer,m_feeder, m_intake));
    Ybutton.whenReleased(new ShootBallsAllStop(m_shooter, m_spindexer,m_feeder, m_intake));

    Bbutton.whenPressed(new IntakeBallsG (m_intake, m_spindexer));
    Bbutton.whenReleased(new IntakeBallsStop(m_intake, m_spindexer));

    Lbutton.whenPressed(new ClimberUp(m_climber)); 
    Lbutton.whenReleased(new ClimberStop(m_climber));    
   
    Rbutton.whenPressed(new ClimberDown(m_climber));
    Rbutton.whenReleased(new ClimberStop(m_climber));    

    Xbutton.whenPressed(new SpoolIn(m_climber));
    Xbutton.whenReleased(new SpoolStop(m_climber));    

    Startbutton.whenPressed(new SpoolOut(m_climber));
    Startbutton.whenReleased(new SpoolStop(m_climber));    

    BackBUtton.whenPressed(new FeederUp(m_feeder));
    BackBUtton.whenReleased(new FeederStop(m_feeder));

    //BackBUtton.whenPressed(new SpindexerSpin(m_spindexer));
    //BackBUtton.whenPressed(new SpindexerStop(m_spindexer));


/*
    Abutton.whenPressed(new IntakeSpin(m_intake)); //stops spindexer
    Bbutton.whenPressed(new SpindexerSpin(m_spindexer));
    Xbutton.whenPressed(new IntakeOut(m_intake));
    Ybutton.whenPressed(new FeederUp(m_feeder)); // not calling 40 not moving pneumatics.
    //Bbutton.whenPressed(new Autonomous1(m_drivetrain));

*/


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return PymblePride.getAutonomousCommand(m_intake, m_drivetrain, m_feeder, m_shooter, m_spindexer);
    // An ExampleCommand will run in autonomous
   // return m_autoCommand;
  }
}
