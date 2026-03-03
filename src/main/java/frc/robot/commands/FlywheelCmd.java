package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Flywheel;

public class FlywheelCmd extends Command {
    private final Flywheel flywheel;
    private final Supplier<Boolean> isFly; 

    public FlywheelCmd(
            Flywheel flywheel,
            Supplier<Boolean> isFly) {

        this.flywheel = flywheel;
        this.isFly = isFly;
        addRequirements(this.flywheel);
    }

    @Override
    public void execute() {
        if (this.isFly.get())
            this.flywheel.setTargetRps(ControllerConstants.FLYWHEEL_RPS); // 例如 80
        else
            this.flywheel.stopFlywheel();
    }

    @Override
    public void end(boolean interrupted) {
        this.flywheel.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
