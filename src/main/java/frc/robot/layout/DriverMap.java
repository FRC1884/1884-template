package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    getPathPlanningTestButton().whenActive(swerve.autoPath(
      "X,Y,Tangent X,Tangent Y,Fixed Theta,Reversed,Name" +
      "3.8922493344077345,-3.214252312651087,0.0,0.0,true,false," +
      "5.238826752618855,-4.048750994359386,0.0,0.0,true,false,"
      ));
  }
}
