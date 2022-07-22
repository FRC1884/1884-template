package frc.util.controllers;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Xbox implements ButtonMap {
    @Override
    public HashMap<Button, Integer> buttonMap() {
        var map = new HashMap<Button, Integer>();

        map.put(Button.BUTTON_A, 0);
        map.put(Button.BUTTON_B, 1);
        map.put(Button.BUTTON_X, 2);
        map.put(Button.BUTTON_Y, 3);
        map.put(Button.BUTTON_LEFT_JOYSTICK, 4);
        map.put(Button.BUTTON_RIGHT_JOYSTICK, 5);
        map.put(Button.BUTTON_LEFT_BUMPER, 6);
        map.put(Button.BUTTON_RIGHT_BUMPER, 7);
        map.put(Button.BUTTON_SHARE, 8);
        map.put(Button.BUTTON_OPTIONS, 9);
        map.put(Button.BUTTON_START, 10);
        map.put(Button.BUTTON_TOUCHPAD, 11);

        return map;
    }

    @Override
    public HashMap<Trigger, Integer> triggerMap() {
        var map = new HashMap<Trigger, Integer>();

        map.put(Trigger.BUTTON_LEFT_TRIGGER, 0);
        map.put(Trigger.BUTTON_RIGHT_TRIGGER, 1);

        return map;
    }

    @Override
    public HashMap<Axis, Integer> axisMap() {
        var map = new HashMap<Axis, Integer>();

        map.put(Axis.AXIS_LEFT_X, 0);
        map.put(Axis.AXIS_LEFT_Y, 1);
        map.put(Axis.AXIS_RIGHT_X, 2);
        map.put(Axis.AXIS_RIGHT_Y, 3);
        map.put(Axis.AXIS_LEFT_TRIGGER, 4);
        map.put(Axis.AXIS_RIGHT_TRIGGER, 5);
        return map;
    }

    @Override
    public HashMap<Dpad, Integer> dpadMap() {
        var map = new HashMap<Dpad, Integer>();

        map.put(Dpad.DPAD_UP, 0);
        map.put(Dpad.DPAD_UP_RIGHT, 45);
        map.put(Dpad.DPAD_RIGHT, 90);
        map.put(Dpad.DPAD_DOWN_RIGHT, 135);
        map.put(Dpad.DPAD_DOWN, 180);
        map.put(Dpad.DPAD_DOWN_LEFT, 225);
        map.put(Dpad.DPAD_LEFT, 270);
        map.put(Dpad.DPAD_UP_LEFT, 315);

        return map;
    }
}