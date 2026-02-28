package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;

public class Feeder extends SubsystemBase  {
    private final SparkFlex leftFeeder;
    private final SparkFlex rightFeeder;
    private final SparkFlex leftSideRoller;
    private final SparkFlex rightSideRoller;
    public Feeder() {
        this.leftFeeder = new SparkFlex(10, MotorType.kBrushless);
        this.rightFeeder = new SparkFlex(17, MotorType.kBrushless);
         this.leftSideRoller = new SparkFlex(21, MotorType.kBrushless);
        this.rightSideRoller = new SparkFlex(23, MotorType.kBrushless);
    }
       public void execute(double speed) {
        this.leftFeeder.set(speed);
        this.rightFeeder.set(-speed);
        this.leftSideRoller.set(-speed);
        this.rightSideRoller.set(speed);
    }

     public Command feed() {
        return Commands.runOnce(() -> this.execute(ControllerConstants.FEEDER_SPEED), this);
    }
    
    public void stopFeeder() {
        this.leftFeeder.stopMotor();
        this.rightFeeder.stopMotor();
        this.leftSideRoller.stopMotor();
        this.rightSideRoller.stopMotor();
    }
}
