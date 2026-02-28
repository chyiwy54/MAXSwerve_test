package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkFlex intake;

    public Intake() {
        this.intake = new SparkFlex(13, MotorType.kBrushless);
    }
       public void execute(double speed) {
        this.intake.set(speed);
    }
    
    public void stopIntake() {
        this.intake.stopMotor();
    }


}