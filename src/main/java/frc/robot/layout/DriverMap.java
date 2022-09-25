package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  @Override
  public void registerCommands() {
    var swerve = SwerveDrive.getInstance();

    swerve.setDefaultCommand(
        swerve.driveCommand(this::getChassisSpeeds));
  }
}
