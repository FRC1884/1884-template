package frc.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class GameController extends Joystick {
  private final double DEADZONE = 0.1;

  private ButtonMap map;

  public GameController(int gamepadPort, ButtonMap map) {
    super(gamepadPort);
    this.map = map;
  }

  public JoystickButton getButton(ButtonMap.Button button) {
    return new JoystickButton(this, map.buttonMap().get(button));
  }

  public JoystickButton getDpad(ButtonMap.Dpad dpad) {
    return new JoystickButton(this, map.dpadMap().get(dpad));
  }

  public double getAxis(ButtonMap.Axis axis) {
    double value = this.getRawAxis(map.axisMap().get(axis));
    if (Math.abs(value) < DEADZONE) {
      return 0;
    } else {
      return value;
    }
  }
}
