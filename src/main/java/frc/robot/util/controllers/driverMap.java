package frc.robot.util.controllers;


public abstract class driverMap extends CommandMap {
  protected GameController driver;

  public driverMap(GameController stick) {
    super(stick);
    driver = stick;
  }
}
