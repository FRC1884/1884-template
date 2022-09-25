package frc.robot.util;

// TODO do we want to move this class somee where else or integrate it into the joysticks?
public class MathUtils {
  /**
   * Kills the signal if it is less than the deadband.
   *
   * @param value The value to be modified.
   * @param deadband The deadband.
   * @return The modified value.
   */
  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Modifies a joystick to make it easier to control for things like drivetrains and lifts and
   * lifts
   *
   * @param value The value to be modified.
   * @return Deadbanded and squared value.
   */
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
