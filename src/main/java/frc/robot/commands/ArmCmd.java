package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmCmd extends Command {
    private final Arm arm;
    private final Supplier<Integer> povSupplier;
    private final double stepSize = 0.8;
     private double targetPosition = 0;

    public ArmCmd(Arm arm,
            Supplier<Integer> povSupplier) {
        this.arm = arm;
        this.povSupplier = povSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        int pov = povSupplier.get();
        
        targetPosition = arm.getTargetPosition();
        // 每一週期 (20ms) 如果按著，就增減目標位置
        // 這裡的 0.5 是移動速度，你可以根據需求調整
        if (pov == 0)// && targetPosition >= (Constants.ControllerConstants.ARM_UP_LIMIT))
        {
            targetPosition -= stepSize; 
        } else if (pov == 180)// && targetPosition <= (Constants.ControllerConstants.ARM_DOWN_LIMIT)) 
        {
            targetPosition += stepSize;
        
        } else if (pov == 90) {
            targetPosition = Constants.ControllerConstants.ARM_DOWN;
        }
        arm.setTargetPosition(targetPosition);  

    }
    

    @Override
    public boolean isFinished() {
        return false;
    }

}