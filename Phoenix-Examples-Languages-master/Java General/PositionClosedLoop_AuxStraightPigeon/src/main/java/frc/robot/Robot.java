/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The PositionClosedLoop_AuxStraightPigeon example demonstrates the new Talon/Victor Auxiliary 
 * and Remote Features to peform more complex. This example has the robot performing Position 
 * Closed Loop with an auxiliary closed loop on Pigeon Yaw to keep the robot straight.
 * 
 * This example uses:
 * - 2x Quad Encoders, One on both sides of robot for Primary Closed Loop on Position
 * A Talon/Victor calculates the distance by taking the sum of both sensors and dividing it by 2.
 * - Pigeon IMU wired on CAN Bus for Auxiliary Closed Loop on Yaw
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Position Closed Loop wtih Quadrature Encoders and Drive Straight Wight Pigeon (Auxiliary)
 * 
 * Controls:
 * Button 1: When pressed, zero all sensors. Set quad encoders' positions + Pigeon yaw to 0.
 * Button 2: When pressed, Toggle between Arcade Drive and Position Closed Loop
 * 	When toggling into Position Closed Loop, the current heading is saved and used as the 
 * 	auxiliary closed loop target. Can be changed by toggling out and in again.
 * Left Joystick Y-Axis: 
 * 	+ Arcade Drive: Drive robot forward and reverse
 * 	+ Position Closed Loop: Servo robot forward and reverse [-6, 6] rotations
 * Right Joystick X-Axis:
 * 	 + Arcade Drive: Turn robot left and right
 * 	+ Position Closed Loop: Not used
 * 
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon SRX: 3.11
 * - Victor SPX: 3.11
 * - Pigeon IMU: 0.42
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Robot extends TimedRobot {
	/** Hardware */
		WPI_TalonFX LeftDrive1 = new WPI_TalonFX(12);
		WPI_TalonFX LeftDrive2 = new WPI_TalonFX(10);
		WPI_TalonFX LeftDrive3 = new WPI_TalonFX(11);
		WPI_TalonFX RightDrive1 = new WPI_TalonFX(5);
		WPI_TalonFX RightDrive2 = new WPI_TalonFX(4);
		WPI_TalonFX RightDrive3 = new WPI_TalonFX(6);


	PigeonIMU _pidgey = new PigeonIMU(0);
	Joystick _gamepad = new Joystick(0);

	TalonFXInvertType LeftInvert = TalonFXInvertType.CounterClockwise;
	TalonFXInvertType RightInvert = TalonFXInvertType.CounterClockwise;
	
	/** Latched values to detect on-press events for buttons */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;

	@Override
	public void robotInit() {
		/* Not used in this example */
	}

	@Override
	public void teleopInit(){
		/* Disable all motor controllers */
		LeftDrive1.set(ControlMode.PercentOutput, 0);
		RightDrive1.set(ControlMode.PercentOutput, 0);
		LeftDrive2.set(ControlMode.PercentOutput, 0);
		RightDrive2.set(ControlMode.PercentOutput, 0);
		LeftDrive3.set(ControlMode.PercentOutput, 0);
		RightDrive3.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		LeftDrive1.configFactoryDefault();
		RightDrive1.configFactoryDefault();
		LeftDrive2.configFactoryDefault();
		RightDrive2.configFactoryDefault();
		LeftDrive3.configFactoryDefault();
		RightDrive3.configFactoryDefault();
		_pidgey.configFactoryDefault();
		
		/* Set Neutral Mode */
		LeftDrive1.setNeutralMode(NeutralMode.Brake);
		RightDrive1.setNeutralMode(NeutralMode.Brake);
		LeftDrive2.setNeutralMode(NeutralMode.Brake);
		RightDrive2.setNeutralMode(NeutralMode.Brake);
		LeftDrive3.setNeutralMode(NeutralMode.Brake);
		RightDrive3.setNeutralMode(NeutralMode.Brake);

		
		LeftDrive2.follow(LeftDrive1);
		RightDrive2.follow(RightDrive1);
		
		LeftDrive3.follow(LeftDrive1);
		RightDrive3.follow(RightDrive1);
		
		
		LeftDrive2.setInverted(InvertType.FollowMaster);
		RightDrive2.setInverted(InvertType.FollowMaster);
		
		LeftDrive3.setInverted(InvertType.FollowMaster);
		RightDrive3.setInverted(InvertType.FollowMaster);

		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		LeftDrive1.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		RightDrive1.configRemoteFeedbackFilter(LeftDrive1.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs);						// Configuration Timeout
		
		/* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		RightDrive1.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												Constants.REMOTE_1,	
												Constants.kTimeoutMs);
		
		/* Setup Sum signal to be used for Distance */
		RightDrive1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);	// Feedback Device of Remote Talon
		RightDrive1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		RightDrive1.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		RightDrive1.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		RightDrive1.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
													Constants.PID_TURN,
													Constants.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		RightDrive1.configSelectedFeedbackCoefficient(	1,
														Constants.PID_TURN,
														Constants.kTimeoutMs);
		
		/* Configure output and sensor direction */
		LeftDrive1.setInverted(false);
		LeftDrive1.setSensorPhase(true);
		RightDrive1.setInverted(true);
		RightDrive1.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		RightDrive1.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		RightDrive1.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		RightDrive1.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		LeftDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		RightDrive1.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		LeftDrive1.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/** 
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		LeftDrive1.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		LeftDrive1.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		RightDrive1.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		RightDrive1.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* FPID Gains for distance servo */
		RightDrive1.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		RightDrive1.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		RightDrive1.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		RightDrive1.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		RightDrive1.config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		RightDrive1.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput, Constants.kTimeoutMs);

		/* FPID Gains for turn servo */
		RightDrive1.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		RightDrive1.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		RightDrive1.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		RightDrive1.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		RightDrive1.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		RightDrive1.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
			
		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        RightDrive1.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
        RightDrive1.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		RightDrive1.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(btns, _gamepad);
		if(btns[2] && !_btns[2]){
			_state = !_state; 		// Toggle state
			_firstCall = true;		// State change, do first call operation
			_targetAngle = RightDrive1.getSelectedSensorPosition(1);
		}else if (btns[1] && !_btns[1]) {
			zeroSensors();			// Zero sensors
		}
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
				
		if(!_state){
			if (_firstCall)
				System.out.println("This is Arcade drive.\n");
			
			LeftDrive1.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			RightDrive1.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (_firstCall) {
				System.out.println("This is Drive Straight Distance with the Auxiliary PID using the Pigeon yaw.");
				System.out.println("Servo [-6, 6] rotations while also maintaining a straight heading.\n");
				zeroDistance();
				
				/* Determine which slot affects which PID */
				RightDrive1.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				RightDrive1.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}
			
			/* Calculate targets from gamepad inputs */
			double target_sensorUnits = forward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
			double target_turn = _targetAngle;
			
			/* Configured for Position Closed loop on Quad Encoders' Sum and Auxiliary PID on Pigeon's Yaw */
			RightDrive1.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, target_turn);
			LeftDrive1.follow(RightDrive1, FollowerType.AuxOutput1);
		}
		_firstCall = false;
	}
	
	/** Zero all sensors, both Pigeon and Talons */
	void zeroSensors() {
		LeftDrive1.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders + Position] All sensors are zeroed.\n");
	}
	
	/** Zero quadrature encoders on Talon */
	void zeroDistance(){
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders] All encoders are zeroed.\n");
	}
	
	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] btns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = gamepad.getRawButton(i);
		}
	}
}
