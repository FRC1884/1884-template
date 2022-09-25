package frc.robot.util.controllers;

import frc.robot.util.controllers.GameController;
import frc.robot.util.controllers.CommandMap;

public abstract class driverMap extends CommandMap {
    protected GameController driver;
    
    public driverMap(GameController stick) {
        super(stick);
        driver = stick;
    }
    
    
}
