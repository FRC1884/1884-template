package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.MoveElevator;
import frc.robot.subsystems.ExampleElevator.Setpoint;

public class LiftAndLowerElevator extends SequentialCommandGroup {
    public LiftAndLowerElevator() {
        addCommands(
            new SequentialCommandGroup(
                new MoveElevator(Setpoint.STATE_3),
                new WaitCommand(5.0),
                new MoveElevator(Setpoint.STATE_1)
        ));
    }
}
