package frc.robot;

import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.ControllerTypes.Logitech;

public class OI {
  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private GameController driver;

  public GameController getDriver() {
    return driver;
  }

  private GameController operator;

  public GameController getOperator() {
    return operator;
  }

  private OI() {
    driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech());
    operator = new GameController(RobotMap.ControllerMap.OPERATOR_JOYSTICK, new Logitech());
  }
}
