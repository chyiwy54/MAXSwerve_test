package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;

public class Shooter extends SubsystemBase {
    private final SparkFlex leftShooter;
    private final SparkFlex rightShooter;

    public Shooter() {
        this.leftShooter = new SparkFlex(14, MotorType.kBrushless);
        this.rightShooter = new SparkFlex(18, MotorType.kBrushless);

    }

    public void execute(double speed) {
        this.leftShooter.set(speed);
        this.rightShooter.set(-speed);
    }

    public void stopShooter() {
        this.leftShooter.stopMotor();
        this.rightShooter.stopMotor();
    }

    public Command shoot() {
        return Commands.runOnce(() -> this.execute(-ControllerConstants.SHOOT_AUTO_SPEED), this);
    }
}
