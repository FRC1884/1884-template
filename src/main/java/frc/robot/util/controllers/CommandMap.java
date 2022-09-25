package frc.robot.util.controllers;

public abstract class CommandMap {

  protected GameController joystick;

  public CommandMap(GameController stick) {
    joystick = stick;
  }

  public abstract void getController();

  public abstract void registerController();
}
