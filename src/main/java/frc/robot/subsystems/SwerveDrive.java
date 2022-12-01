package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
// import com.omagarwal25.swervelib.Mk4SwerveModuleHelper;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
// import com.omagarwal25.swervelib.SdsModuleConfigurations;
// import com.omagarwal25.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.DriveMap;
import frc.robot.RobotMap.DriveMap.Version;
import java.util.HashMap;
import java.util.function.Supplier;

public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive instance;

  public static SwerveDrive getInstance() {
    if (instance == null) instance = new SwerveDrive();
    return instance;
  }

  /**
   * The maximum voltage that will be delivered to the drive motors.
   *
   * <p>This can be reduced to cap the robot's maximum speed. Typically, this is useful during
   * initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk4 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      DriveMap.DRIVE_MOTOR_FREE_SPEED
          / 60.0
          * DriveMap.getModuleConfiguration().getDriveReduction()
          * DriveMap.getModuleConfiguration().getWheelDiameter()
          * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(DriveMap.TRACKWIDTH_METERS / 2.0, DriveMap.WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(DriveMap.TRACKWIDTH_METERS / 2.0, DriveMap.WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DriveMap.TRACKWIDTH_METERS / 2.0, -DriveMap.WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DriveMap.TRACKWIDTH_METERS / 2.0, DriveMap.WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DriveMap.TRACKWIDTH_METERS / 2.0, -DriveMap.WHEELBASE_METERS / 2.0));

  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DriveMap.PIGEON_ID);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private SwerveModuleState[] states;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveDriveOdometry swerveDriveOdometry;

  private SwerveModule createModule(
      DriveMap.Module module, ShuffleboardTab tab, int shuffleboardColumn, String name) {
    var drive = module.getDriveId();
    var steer = module.getSteerId();
    var encoder = module.getEncoderId();
    var offset = module.getSteerOffset();

    var layout = tab.getLayout(name).withSize(2, 4).withPosition(shuffleboardColumn, 0);

    if (DriveMap.VERSION == Version.MK3) {
      return Mk3SwerveModuleHelper.createFalcon500(
          layout, DriveMap.MK3_GEAR_RATIO, drive, steer, encoder, offset);
    } else {
      return Mk4SwerveModuleHelper.createFalcon500(
          layout, DriveMap.MK4_GEAR_RATIO, drive, steer, encoder, offset);
    }
  }

  private SwerveDrive() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    frontLeftModule = createModule(DriveMap.FRONT_LEFT_MODULE, tab, 0, "Front Left Module");
    frontRightModule = createModule(DriveMap.FRONT_RIGHT_MODULE, tab, 2, "Front Right Module");
    backRightModule = createModule(DriveMap.BACK_RIGHT_MODULE, tab, 4, "Back Right Module");
    backLeftModule = createModule(DriveMap.BACK_LEF_MODULE, tab, 6, "Back Left Module");

    swerveDriveOdometry = new SwerveDriveOdometry(kinematics, this.getGyroscopeRotation());
    setSwerveModuleState(chassisSpeeds);
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently
   * facing to the 'forwards' direction.
   */
  public void zeroGyroscope() {
    pigeon.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    // Glass widget for the gyroscope
    SmartDashboard.putData("Pigeon2 rotation", pigeon);

    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public void drive(ChassisSpeeds newChassisSpeeds) {
    chassisSpeeds = newChassisSpeeds;
    setSwerveModuleState(chassisSpeeds);
  }

  public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return new PerpetualCommand(new RunCommand(() -> drive(chassisSpeedsSupplier.get()), this));
  }

  public SequentialCommandGroup followTrajectoryCommand(String path, boolean isFirstPath) {
    return followTrajectoryCommand(path, new HashMap<>(), isFirstPath);
  }

  public SequentialCommandGroup followTrajectoryCommand(
      String path, HashMap<String, Command> eventMap, boolean isFirstPath) {
    PathPlannerTrajectory traj = PathPlanner.loadPath(path, 1, 1);

    // Create PIDControllers for each movement (and set default values)
    PIDController xPID = new PIDController(0.1, 0.0, 0.0);
    PIDController yPID = new PIDController(0.1, 0.0, 0.0);
    PIDController thetaPID = new PIDController(0.1, 0.0, 0.0);

    // Create PID tuning widgets in Glass (not for use in competition)
    SmartDashboard.putData("x-input PID Controller", xPID);
    SmartDashboard.putData("y-input PID Controller", yPID);
    SmartDashboard.putData("rot PID Controller", thetaPID);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                swerveDriveOdometry.resetPosition(
                    traj.getInitialHolonomicPose(), getGyroscopeRotation());
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose,
            kinematics,
            xPID,
            yPID,
            thetaPID,
            this::setSwerveModuleState,
            eventMap,
            this));
  }

  public void setSwerveModuleState(SwerveModuleState[] s) {
    states = s;
  }

  public void setSwerveModuleState(ChassisSpeeds s) {
    states = kinematics.toSwerveModuleStates(s);
  }

  @Override
  public void periodic() {
    // zeroGyroscope();
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    swerveDriveOdometry.update(getGyroscopeRotation(), states);

    frontLeftModule.set(
        states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());
    frontRightModule.set(
        states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    backLeftModule.set(
        states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());
    backRightModule.set(
        states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());
  }
}
