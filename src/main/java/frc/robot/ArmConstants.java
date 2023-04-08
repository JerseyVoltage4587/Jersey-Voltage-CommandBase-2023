/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class ArmConstants {

	public static final double kHighCone = 170000;
	public static final double kMidCone = 120000;
	public static final double kGrabCone = 160000;
	public static final double kHighCube = 186000;
	public static final double kMidCube = 130000;
	public static final double kGrabCube = 195000;
	public static final double kResting = 40000;

	public static final double kExtend = 0.6;
	public static final double kRetract = -0.6;




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

	public static final double adjScale = 20000;

	public static final double kCruiseVelocity = 24000;
	public static final double kCruiseAccel = 24000;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(0.15, 0.0000, 0.0, 0.0313, 1000, 1.0);
	public static int kSmooth = 2;
}
