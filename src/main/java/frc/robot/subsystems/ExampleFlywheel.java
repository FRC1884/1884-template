package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;

public class ExampleFlywheel extends SubsystemBase {
  private static ExampleFlywheel instance;

  public static ExampleFlywheel getInstance() {
    if (instance == null) instance = new ExampleFlywheel();
    return instance;
  }

  // Motor Controllers
  private CANSparkMax leaderFlywheel, followerFlywheel;

  // Camera
  private PhotonCamera vision;

  // PID Controllers and Gains
  // PID can be tuned in REV Hardware client
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private SparkMaxPIDController leader_pidController;
  private SparkMaxPIDController follower_pidController;

  private ExampleFlywheel() {
    leaderFlywheel = new CANSparkMax(RobotMap.FlywheelMap.LEADER_FLYWHEEL, MotorType.kBrushless);
    followerFlywheel =
        new CANSparkMax(RobotMap.FlywheelMap.FOLLOWER_FLYWHEEL, MotorType.kBrushless);
    vision = new PhotonCamera(RobotMap.CameraMap.COMPUTER_VISION);

    leaderFlywheel.setInverted(
        false); // In case one of the motors needs to be invrted before setting them to follow
    followerFlywheel.follow(leaderFlywheel, false);

    // leader_pidController = ;
    // follower_pidController = ;
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 0;
    kMinOutput = 0;
  }
}
