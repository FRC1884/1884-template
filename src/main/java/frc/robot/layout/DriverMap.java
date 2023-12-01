package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  @Override
  public void registerCommands() {
  }
}
