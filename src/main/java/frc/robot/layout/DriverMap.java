package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getPathPlanningTestButton();

  @Override
  public void registerCommands() {
    var swerve = SwerveDrive.getInstance();

    swerve.setDefaultCommand(swerve.driveCommand(this::getChassisSpeeds));

    System.out.println(Filesystem.getDeployDirectory().listFiles()[0].listFiles()[0].getName() + "YOUR MOM IS MINE");
    getPathPlanningTestButton().whenActive(swerve.autoPath("Old Way"));
  }
}
