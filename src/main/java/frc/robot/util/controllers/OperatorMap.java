package frc.robot.util.controllers;


public abstract class OperatorMap extends CommandMap {
  protected GameController operator;

  public OperatorMap(GameController controller) {
    super(controller);
    operator = controller;
  }

  public abstract void getElevatorUpButton();

  public abstract void getElevatorDownButton();

  public abstract void getElevatorMiddleButton();
}
