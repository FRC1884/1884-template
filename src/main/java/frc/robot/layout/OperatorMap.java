package frc.robot.layout;

import frc.robot.util.controllers.CommandMap;
import frc.robot.util.controllers.GameController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  public abstract void getElevatorUpButton();

  public abstract void getElevatorDownButton();

  public abstract void getElevatorMiddleButton();
}
