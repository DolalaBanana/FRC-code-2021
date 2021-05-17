/**
 * Simple class containing constants used throughout project
 */
package frc.robot.subsystems.Shooter;

class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
	
	 //40% speed - 2552*2048/600
    public final static Gains kGains_Velocit40 = new Gains((0.125*1023)/1043, 0, 10*((0.125*1023)/1043), (0.4*1023)/8024,  300,  1.00);
	 
	public final static Gains kGains_Velocit50 = new Gains( (0.1*1023)/1123, 0, 10*(0.1*1023)/1123, (0.5*1023)/9585,  300,  1.00);

	public final static Gains kGains_Velocit60 = new Gains( (0.2*1023)/1385, 0, 10*(0.2*1023)/1385, (0.6*1023)/11759,  300,  1.00);
	// 80% speed - 5100 * 2048/600
	public final static Gains kGains_Velocit80 = new Gains( (0.82*1023)/5193, 0, 100*((0.82*1023)/5193), (0.8*1023)/16182,  300,  1.00);
}
//(0.82*1023)/5193