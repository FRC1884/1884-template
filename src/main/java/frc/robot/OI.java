package frc.robot;

import frc.robot.RobotMap;
import frc.robot.util.controllers.*;

public class OI {
  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private GameController driver;
  private GameController operator;

  private OI() {
    driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech());
    operator = new GameController(RobotMap.ControllerMap.OPERATOR_JOYSTICK, new Logitech());
  }
}
