package frc.robot.commands;
import java.util.function.Supplier;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;
public class FeederCmd extends Command {
    private final Feeder feeder;
    private final Supplier<Boolean> isFeed;
    private final Supplier<Boolean> isOutFeed;

    public FeederCmd(Feeder feeder,
            Supplier<Boolean> isFeed,
            Supplier<Boolean> isOutFeed) {
        this.feeder = feeder;
        this.isFeed = isFeed;
        this.isOutFeed = isOutFeed;
        addRequirements(this.feeder);
    }

    @Override
    public void execute() {
        if (this.isFeed.get())
            this.feeder.execute(ControllerConstants.FEEDER_SPEED);
        else if (this.isOutFeed.get())
            this.feeder.execute(-ControllerConstants.OUTFEED_SPEED);
        else
            this.feeder.stopFeeder();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end(boolean interrupted) {
        this.feeder.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
    

