package frc.robot.layout;

import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract void getElevatorUpButton();

  public abstract void getElevatorDownButton();

  public abstract void getElevatorMiddleButton();
}
