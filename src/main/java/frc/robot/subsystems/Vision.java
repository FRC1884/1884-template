package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) instance = new Vision();
    return instance;
  }

  // Camera
  private PhotonCamera vision;

  // Devil's in the details
  private PhotonTrackedTarget latestTarget;

  public Vision() {
    vision = new PhotonCamera(RobotMap.CameraMap.COMPUTER_VISION);
  }

  private void updateResult() {
    if (vision.getLatestResult().hasTargets())
      latestTarget = vision.getLatestResult().getBestTarget();
  }

  public double getAngle() {
    return latestTarget.getYaw();
  }

  public double getRange() {
    // return latestTarget.
  }

  @Override
  public void periodic() {
    updateResult();
  }
}
