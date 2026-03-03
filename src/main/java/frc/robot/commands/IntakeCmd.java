package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCmd extends Command {
    private final Intake intake;
    private final Supplier<Double> isIntake;
    private final Supplier<Boolean> isOuttake;

    public IntakeCmd(Intake intake,
            Supplier<Double> isIntake,
            Supplier<Boolean> isOuttake) {
        this.intake = intake;
        this.isIntake = isIntake;
        this.isOuttake = isOuttake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        double it = this.isIntake.get(); 
        if (it > 0.05) {
            this.intake.execute(ControllerConstants.INTAKE_SPEED * it);
        } else if (isOuttake.get()) {
            this.intake.execute(-ControllerConstants.OUTTAKE_SPEED);
        }
        else {
            this.intake.stopIntake();
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        this.intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
