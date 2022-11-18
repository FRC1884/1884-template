package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;
import java.util.HashMap;

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

    var commands = new HashMap<String, Command>();
    commands.put(
        "marker",
        new InstantCommand(
            () -> {
              System.out.println("Hello Lamine");
            }));

    getPathPlanningTestButton()
        .whenActive(swerve.followTrajectoryCommand("Old Way", commands, true));
  }
}
