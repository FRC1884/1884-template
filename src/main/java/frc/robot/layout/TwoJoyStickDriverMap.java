package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public JoystickButton getTestButton(){
    return controller.getButton(Button.BUTTON_B);
  }
  
  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
