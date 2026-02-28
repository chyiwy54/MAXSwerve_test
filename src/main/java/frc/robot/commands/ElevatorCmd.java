package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorCmd extends Command {
    private final Elevator elevator;
    private final Supplier<Boolean> isDown;
    private final Supplier<Boolean> isUp;
    private final Supplier<Boolean> L1;
    //private final Supplier<Boolean> L3;
    private final Supplier<Boolean> Lowest;
    private final double stepSize = 2.0;

    private double targetPosition = 0;

    public ElevatorCmd(
            Elevator elevator,
            Supplier<Boolean> isUp,
            Supplier<Boolean> isDown,
            Supplier<Boolean> L1,
            //Supplier<Boolean> L3,
            Supplier<Boolean> Lowest
    ) {
        this.elevator = elevator;
        this.isDown = isUp;
        this.isUp = isDown;
        this.L1 = L1;
        //this.L3 = L3;
        this.Lowest = Lowest;
        addRequirements(elevator);
    }
    @Override
    public void execute() {
        if (isDown.get() )//&& targetPosition >= (Constants.ControllerConstants.ELEVATOR_DOWN_LIMIT))
            targetPosition += stepSize;
        else if (isUp.get())// && targetPosition <= (Constants.ControllerConstants.ELEVATOR_UP_LIMIT))
                targetPosition -= stepSize;
        else if (L1.get())
            targetPosition = (Constants.ControllerConstants.L1);
        else if (Lowest.get())
            targetPosition = (Constants.ControllerConstants.LOWEST);
        elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevator.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}