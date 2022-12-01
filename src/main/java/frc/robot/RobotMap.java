package frc.robot;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

public class RobotMap {
  public static class ElevatorMap {
    public static final int master = 4;
    public static final int slave = 5;

    public static final int limitSwitch = 0;
  }

  public static class DriveMap {

    public static final double DRIVE_MOTOR_FREE_SPEED = 6380;

    public static class Module {
      public Module(int driveId, int steerId, int encoderId, double steerOffsetDegrees) {
        this.driveId = driveId;
        this.steerId = steerId;
        this.encoderId = encoderId;
        this.steerOffsetDegrees = steerOffsetDegrees;
      }

      private int driveId;

      public int getDriveId() {
        return driveId;
      }

      private int steerId;

      public int getSteerId() {
        return steerId;
      }

      private int encoderId;

      public int getEncoderId() {
        return encoderId;
      }

      private double steerOffsetDegrees;

      public double getSteerOffset() {
        return -Math.toRadians(steerOffsetDegrees);
      }
    }

    public enum Version {
      MK3,
      MK4
    }

    public static final Version VERSION = Version.MK4;

    public static final ModuleConfiguration MK3_MODULE_CONFIGURATION =
        SdsModuleConfigurations.MK3_STANDARD;
    public static final ModuleConfiguration MK4_MODULE_CONFIGURATION =
        SdsModuleConfigurations.MK4_L1;

    public static ModuleConfiguration getModuleConfiguration() {
      return VERSION == Version.MK3 ? MK3_MODULE_CONFIGURATION : MK4_MODULE_CONFIGURATION;
    }

    public static final Mk3SwerveModuleHelper.GearRatio MK3_GEAR_RATIO =
        Mk3SwerveModuleHelper.GearRatio.STANDARD;
    public static final Mk4SwerveModuleHelper.GearRatio MK4_GEAR_RATIO =
        Mk4SwerveModuleHelper.GearRatio.L1;

    public static final double TRACKWIDTH_METERS = 0.617; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * <p>Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.617; // FIXME Measure and set wheelbase

    public static final int PIGEON_ID = 9;

    public static final Module FRONT_LEFT_MODULE = new Module(7, 8, 18, 270);

    public static final Module FRONT_RIGHT_MODULE = new Module(1, 2, 17, 260);

    public static final Module BACK_LEF_MODULE = new Module(5, 6, 16, 270);

    public static final Module BACK_RIGHT_MODULE = new Module(3, 4, 19, 265);
  }

  public static class TankDriveMap {
    public static final int leftFrontMaster = 0;
    public static final int leftBackMaster = 1;
    public static final int rightFrontMaster = 2;
    public static final int rightBackMaster = 3;
  }

  public static class FlywheelMap {
    public static final int LEADER_FLYWHEEL = -1; // FIXME Set flywheel motor ID
    public static final int FOLLOWER_FLYWHEEL = -1; // FIXME Set flywheel motor ID
  }

  public static class CameraMap {
    // Rename the cameras in phtonvision dashboard to the corresponding camera name
    public static final String COMPUTER_VISION = "camscanner";
    public static final String DRIVER_CAMERA = "drivercam";
  }

  public static class ControllerMap {
    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
  }
}
