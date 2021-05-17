// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DrivetrainSub;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public class Drivetrain extends SubsystemBase {

  /** Hardware */
	WPI_TalonFX _leftMaster = new WPI_TalonFX(3);
	WPI_TalonFX _leftFollower = new WPI_TalonFX(4);

	WPI_TalonFX _rightMaster = new WPI_TalonFX(6);
	WPI_TalonFX _rightFollower = new WPI_TalonFX(5);

	PigeonIMU _pidgey = new PigeonIMU(0);
	Joystick _gamepad = new Joystick(0);

  DoubleSolenoid _shifter = new DoubleSolenoid(6,7); 
;

  DifferentialDrive m_safety_drive = new DifferentialDrive(_leftMaster, _rightMaster);

	double maxVel = 0;
	public final static double counts_per_CM = (2048*9.17)/(10.16*3.14159265359); 

	/** Invert Directions for Left and Right */
	TalonFXInvertType _rightInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false" 
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "true"
	
	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	

	double _targetAngle = 0;

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    robotInit();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void drive(double speed, double rotation) {
	  System.out.println("driving..." + speed + " , " + rotation);
    m_safety_drive.arcadeDrive(speed,rotation);
  }


  public void driveauto(){
	m_safety_drive.arcadeDrive(0.6,0);
  }
  public void stopDrive(){
	_leftMaster.set(0);
	_rightMaster.set(0);
	_leftFollower.set(0);
	_rightFollower.set(0);
  }

  public void HighGear(){
	_shifter.set(Value.kForward);
}

public void LowGear(){
  _shifter.set(Value.kReverse);
}
  
  public void move(double dist){
				
				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			
			
			/* Calculate targets from gamepad inputs */
			double target_sensorUnits = dist*counts_per_CM; // forward* Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel*9.17; 
			double target_turn = 0;//900 for 90 degrees;

			/* Configured for MotionMagic on Integrated Encoders' Sum and Auxiliary PID on Pigeon */
			
			_rightMaster.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
      _leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
      
      SmartDashboard.putNumber("error", _rightMaster.getClosedLoopError(0));

  }

  public void turn(double angle){
    	/* Determine which slot affects which PID */
      _rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
      _rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
    
    
    /* Calculate targets from gamepad inputs */
    double target_sensorUnits = 0; // forward* Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel*9.17; 
    double target_turn = angle*10;//900;

    /* Configured for MotionMagic on Integrated Encoders' Sum and Auxiliary PID on Pigeon */
    
    _rightMaster.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
    _leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
    
    //SmartDashboard.putNumber("error", _rightMaster.getClosedLoopError(0));

  }
  public void stopDrivetrain(){
    _leftMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
			_rightMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
  }
  
  public boolean hasreachedTarget(){

    if(Math.abs(_rightMaster.getClosedLoopTarget(0))- Math.abs(_rightMaster.getSelectedSensorPosition()) < Math.abs(_rightMaster.getClosedLoopTarget(0))*0.02 ){
      return true;
    }else {
      return false; 
    }
  }


public void robotInit(){

	_leftMaster.configFactoryDefault();
	_leftFollower.configFactoryDefault();
	_rightMaster.configFactoryDefault();
	_rightMaster.configFactoryDefault();

  /* Set Neutral Mode */
  _leftMaster.setNeutralMode(NeutralMode.Brake);
  _rightMaster.setNeutralMode(NeutralMode.Brake);

  _leftFollower.setNeutralMode(NeutralMode.Brake);
  _rightFollower.setNeutralMode(NeutralMode.Brake);

  /* Configure output and sensor direction */
  _leftMaster.setInverted(_leftInvert);
  _rightMaster.setInverted(_rightInvert);

  _leftFollower.setInverted(_leftInvert);
  _rightFollower.setInverted(_rightInvert);
  
  _leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  _rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

_leftMaster.configOpenloopRamp(0.3);
_rightMaster.configOpenloopRamp(0.3);
_leftFollower.configOpenloopRamp(0.3);
_rightFollower.configOpenloopRamp(0.3);

  //Followers
  _leftFollower.follow(_leftMaster);
  _rightFollower.follow(_rightMaster);

}

double Deadband(double value) {
  /* Upper deadband */
  if (value >= +0.1) 
    return value;
  
  /* Lower deadband */
  if (value <= -0.1)
    return value;
  
  /* Outside deadband */
  return 0;
}


	/** Zero all sensors, both Talons and Pigeon */
	public void zeroSensors() {
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_leftFollower.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightFollower.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);

		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Integrated Encoders + Pigeon] All sensors are zeroed.\n");
	}
	
	/** Zero Integrated Encoders, used to reset position when initializing Motion Magic */
	void zeroDistance(){
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		System.out.println("[Integrated Encoders] All encoders are zeroed.\n");
	}
	
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] _currentBtns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			_currentBtns[i] = gamepad.getRawButton(i);
		}
	}

	/** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	 void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.

				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.

				Will a sensor sum or difference give us a positive total magnitude?

				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.

					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }
}
