package frc.robot;

<<<<<<< HEAD
import frc.robot.RobotMap;
import frc.robot.util.controllers.*;
=======
import frc.util.controllers.GameController;
import frc.util.controllers.Logitech;
>>>>>>> c57ba2912b8634cef73ce58c62dfbcbe57fb2bb1

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
