package frc.robot;

import frc.util.controllers.*;
import frc.robot.RobotMap;
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

  private OI(){
  }
}
