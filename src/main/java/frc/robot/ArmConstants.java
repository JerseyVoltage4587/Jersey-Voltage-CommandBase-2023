/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class ArmConstants {

	public static final double kHighCone = 18000;
	public static final double kMidCone = 17000;
	public static final double kGrabCone = 16000;
	public static final double kHighCube = 15000;
	public static final double kMidCube = 14000;
	public static final double kGrabCube = 13000;
	public static final double kResting = 1000;


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
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	public static final double adjScale = 100;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(0.001, 0.0, 0.0, 0.0313, 0, 1.0);
	public static int kSmooth = 2;
}
