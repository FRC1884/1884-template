package frc.robot.util.controllers;

public abstract class DriverMap extends CommandMap {
  protected GameController driver;

  public DriverMap(GameController controller) {
    super(controller);
    driver = controller;
  }
}
