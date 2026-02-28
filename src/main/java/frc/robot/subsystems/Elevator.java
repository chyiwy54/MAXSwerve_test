package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator extends SubsystemBase {
    private final SparkFlex elevator;
    private final PIDController elevatorPid;
    private final RelativeEncoder encoder;
    private double targetPosition = 0; 

    public Elevator() {
        this.elevator = new SparkFlex(20, MotorType.kBrushless);
        this.encoder = elevator.getEncoder();
        this.elevatorPid = new PIDController(0.1, 0.0, 0);
        this.encoder.setPosition(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getTargetPosition() {
        return this.targetPosition;
    }

    public void setElevatorPosition(double position) {
        this.targetPosition = position;
    }

    public void updateElevator() {
        elevator.set(elevatorPid.calculate(encoder.getPosition(), this.targetPosition));
    }

    public void stopElevator() {
        this.elevator.stopMotor();
    }

    public Command moveToL1() {
        return Commands.runOnce(() -> {this.targetPosition = ControllerConstants.L1; }, this);
    }

    public Command moveToLowest() {
        return Commands.runOnce(() -> {this.targetPosition = ControllerConstants.LOWEST; }, this);
    }

    @Override
    public void periodic() {
        updateElevator();
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Target Position", targetPosition);
    }
}