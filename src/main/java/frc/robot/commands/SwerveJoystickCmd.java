package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> isAutoTurning;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, 
            Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> isAutoTurning
        ) {
         this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.isAutoTurning = isAutoTurning;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        
       if (Robot.isSimulation()) {
            this.swerveSubsystem.simDrive(this.xSpdFunction.get(), this.ySpdFunction.get(), this.turningSpdFunction.get());
            return;
        }
        
        if (this.isAutoTurning.get()) {
            this.swerveSubsystem.autoTurning(this.xSpdFunction.get(), this.ySpdFunction.get(), Vision.getRightTx());
        } else {
            this.swerveSubsystem.driveSwerve(this.xSpdFunction.get(), this.ySpdFunction.get(), this.turningSpdFunction.get());
        }
    }
@Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


