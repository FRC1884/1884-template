package frc.robot;

public class RobotMap {
  public static class ElevatorMap {
    public static final int master = 4;
    public static final int slave = 5;

    public static final int limitSwitch = 0;
  }

  public static class DriveMap {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS =
        0.617; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS =
        0.617; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 9; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR =
        3; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR =
        4; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER =
        13; // FIXME Set front left steer encoder ID

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
        -Math.toRadians(340); // FIXME Measure and set front
    // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR =
        1; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR =
        2; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER =
        10; // FIXME Set front right steer encoder ID

    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
        -Math.toRadians(270); // FIXME Measure and set front
    // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER =
        11; // FIXME Set back left steer encoder ID

    public static final double BACK_LEFT_MODULE_STEER_OFFSET =
        -Math.toRadians(315.0); // FIXME Measure and set back left
    // steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR =
        7; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR =
        8; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER =
        12; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
        -Math.toRadians(60); // FIXME Measure and set back
    // right steer offset
  }

  public static class TankDriveMap {
    public static final int leftFrontMaster = 0;
    public static final int leftBackMaster = 1;
    public static final int rightFrontMaster = 2;
    public static final int rightBackMaster = 3;
  }

  public static class ControllerMap {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
  }
}
