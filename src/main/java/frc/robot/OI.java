package frc.robot;

import frc.robot.layout.TwoJoyStickDriverMap;
import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.Logitech;

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

  public void registerCommands() {
    new TwoJoyStickDriverMap(driver).registerCommands();
    // operator.registerCommands();
  }

  private OI() {
    // driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech());
    driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech(1));
    // operator = new GameController(RobotMap.ControllerMap.OPERATOR_JOYSTICK, new Logitech());
  }
}
