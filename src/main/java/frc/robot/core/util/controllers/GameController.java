package frc.robot.core.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.MathUtils;

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

    return MathUtils.signSquare(MathUtils.deadband(value, DEADZONE));
  }

  public double getTrigger(ButtonMap.Trigger trigger) {
    return this.getRawAxis(map.triggerMap().get(trigger));
  }

  public int getDpadAngle() {
    return this.getPOV();
  }

}
