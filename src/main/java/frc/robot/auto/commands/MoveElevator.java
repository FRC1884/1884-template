package frc.robot.auto.commands;

import frc.robot.subsystems.ExampleElevator;
import frc.robot.subsystems.ExampleElevator.Setpoint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class MoveElevator extends SequentialCommandGroup {
    private final ExampleElevator mElevator = ExampleElevator.getInstance();

    public MoveElevator(Setpoint desiredPosition) {
        mElevator.moveElevatorCommand(desiredPosition);

    }
}
