package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class ShooterCmd extends Command {
    private final Shooter shooter;
    private final Supplier<Double> isShoot;

    public ShooterCmd(Shooter shooter,
            Supplier<Double> isShoot) {
        this.shooter = shooter;
        this.isShoot = isShoot;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double rt = isShoot.get();
        if (rt > 0.05) {
            this.shooter.execute(ControllerConstants.SHOOTER_SPEED);
        } else {
            this.shooter.stopShooter();
        }
    }
    

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}