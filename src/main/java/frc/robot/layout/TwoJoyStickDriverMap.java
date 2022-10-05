package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.ButtonMap.Button;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    var y = controller.getAxis(Axis.AXIS_LEFT_X) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * 0.1;
    var x = controller.getAxis(Axis.AXIS_LEFT_Y) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * 0.1;
    var rot =
        controller.getAxis(Axis.AXIS_RIGHT_X)
            * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            * 0.1;

    var swerve = SwerveDrive.getInstance();
    return ChassisSpeeds.fromFieldRelativeSpeeds(-x, -y, -rot, swerve.getGyroscopeRotation());
  }

  @Override
  public JoystickButton getPathPlanningTestButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
