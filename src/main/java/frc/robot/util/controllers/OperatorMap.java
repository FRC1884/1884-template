package frc.robot.util.controllers;

import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.CommandMap;

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
