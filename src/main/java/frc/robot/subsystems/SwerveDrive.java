package frc.robot.subsystems;

import static frc.robot.RobotMap.DriveMap.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.RobotMap.DriveMap.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.RobotMap.DriveMap.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.RobotMap.DriveMap.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.RobotMap.DriveMap.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.RobotMap.DriveMap.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.RobotMap.DriveMap.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.RobotMap.DriveMap.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.RobotMap.DriveMap.DRIVETRAIN_PIGEON_ID;
import static frc.robot.RobotMap.DriveMap.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.RobotMap.DriveMap.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.RobotMap.DriveMap.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.RobotMap.DriveMap.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.RobotMap.DriveMap.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.RobotMap.DriveMap.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.RobotMap.DriveMap.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.RobotMap.DriveMap.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.RobotMap.DriveMap.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.RobotMap.DriveMap.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
// import com.omagarwal25.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
// import com.omagarwal25.swervelib.SdsModuleConfigurations;
// import com.omagarwal25.swervelib.SwerveModule;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
      6380.0
          / 60.0
          * SdsModuleConfigurations.MK4_L1.getDriveReduction()
          * SdsModuleConfigurations.MK4_L1.getWheelDiameter()
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
          / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(
              -DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DRIVETRAIN_PIGEON_ID);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private SwerveModuleState[] states;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private SwerveDriveOdometry swerveDriveOdometry;

  private SwerveDrive() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    frontLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of
            // the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    frontRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET);

    backLeftModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET);

    backRightModule =
        Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET);

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
    pigeon.setFusedHeading(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    // Glass widget for the gyroscope
    SmartDashboard.putData("PigeonIMU rotation", pigeon);

    return Rotation2d.fromDegrees(pigeon.getFusedHeading());
  }

  public void drive(ChassisSpeeds newChassisSpeeds) {
    chassisSpeeds = newChassisSpeeds;
    setSwerveModuleState(chassisSpeeds);
  }

  public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return new PerpetualCommand(new RunCommand(() -> drive(chassisSpeedsSupplier.get()), this));
  }

  public PPSwerveControllerCommand autoPath(String path) {
    PathPlannerTrajectory autoTestPath = PathPlanner.loadPath(path, 1, 1);

    // Create PIDControllers for each movement (and set default values)
    PIDController xPID = new PIDController(0.1, 0.0, 0.0);
    PIDController yPID = new PIDController(0.1, 0.0, 0.0);
    ProfiledPIDController thetaPID =
        new ProfiledPIDController(10, 0.0, 0.0, new TrapezoidProfile.Constraints(1, 1));

    // Create PID tuning widgets in Glass (not for use in competition)
    SmartDashboard.putData("x-input PID Controller", xPID);
    SmartDashboard.putData("y-input PID Controller", yPID);
    SmartDashboard.putData("rot PID Controller", thetaPID);

    return new PPSwerveControllerCommand(
        autoTestPath,
        this::getPose,
        kinematics,
        xPID,
        yPID,
        thetaPID,
        this::setSwerveModuleState,
        this);
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
