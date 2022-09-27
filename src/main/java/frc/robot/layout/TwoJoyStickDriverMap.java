package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.controllers.ButtonMap.Axis;
import frc.robot.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    var y = controller.getAxis(Axis.AXIS_LEFT_X) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * 0.15;
    var x = controller.getAxis(Axis.AXIS_LEFT_Y) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND * 0.15;
    var rot =
        controller.getAxis(Axis.AXIS_RIGHT_X)
            * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1;

    return new ChassisSpeeds(-x, -y, -rot);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
