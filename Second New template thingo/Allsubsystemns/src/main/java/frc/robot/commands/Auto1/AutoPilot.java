/**
* This is a very simple command program that simulates robot arcade drive
* without using complicated PID control over the distance.
* This is made for an unspecified length of auto drive during competition 
* autonomous period. The driving is designed to take 4 - 5 seconds.
*/
package frc.robot.commands.Auto1;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSub.Drivetrain;

public class AutoPilot extends CommandBase {
    private Drivetrain m_driver;
    private static double initialSpeed = 0.0;
    private static double initialAngle = 0.0;
    private static double rampRate = 0.5;
    private static double rampTurn = 0.015;
 
    /** Creates a new Shooter40. */
    public AutoPilot(Drivetrain driver) {
        m_driver = driver;
        addRequirements(m_driver);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        try {
            // 2 seconds of turning
            double prev_angle = initialAngle;
            double turn;
            for (int i = 0; i < 20; i++) {
                turn = prev_angle + rampTurn * 0.05;
                m_driver.drive(0.0, turn);
                prev_angle = turn; System.out.println("Turn : " + turn);
                Thread.sleep(50);
            }
            for (int i = 0; i < 20; i++) {
                turn = prev_angle - rampTurn * 0.05;
                m_driver.drive(0.0, turn); System.out.println("Turn : " + turn);
                prev_angle = turn;
                Thread.sleep(50);
            }

            double prev_speed = initialSpeed;
            double speed;
            // 2.5 seconds of driving
            for (int i = 0; i < 25; i++) {
                speed = prev_speed + (rampRate * 0.05)/12;
                m_driver.drive(speed, 0.0); 
                prev_speed = speed; System.out.println("Speed : " + speed);
                Thread.sleep(50);
            } 
            for (int i = 0; i < 25; i++) {
                speed = prev_speed - (rampRate * 0.05)/12;
                m_driver.drive(speed, 0.0);
                prev_speed = speed; System.out.println("Speed : " + speed);
                Thread.sleep(50);
            } 
        }
        catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
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
