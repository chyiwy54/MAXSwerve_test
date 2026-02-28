package frc.robot.commands;

import java.util.function.Supplier;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCmd extends Command {
    private final Intake intake;
    private final Supplier<Boolean> isIntake;
    private final Supplier<Boolean> isIntake2;
    private final Supplier<Double> isOuttake;

    public IntakeCmd(Intake intake,
            Supplier<Boolean> isIntake,
            Supplier<Boolean> isIntake2,
            Supplier<Double> isOuttake) {
        this.intake = intake;
        this.isIntake = isIntake;
        this.isIntake2 = isIntake2;
        this.isOuttake = isOuttake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        double lt = isOuttake.get();
        if (this.isIntake.get()) {
            this.intake.execute(ControllerConstants.INTAKE_SPEED);
        } else if (this.isIntake2.get()) {
            this.intake.execute(ControllerConstants.INTAKE2_SPEED);
        } else if (lt > 0.05) {
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
