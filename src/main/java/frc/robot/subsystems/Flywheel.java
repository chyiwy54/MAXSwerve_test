package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

    private final VelocityVoltage m_velReq = new VelocityVoltage(0);
    private final TalonFX leftFlywheel;
    private final TalonFX rightFlywheel;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temp;

    public Flywheel() {
        leftFlywheel = new TalonFX(15);
        rightFlywheel = new TalonFX(16);

        // ===== 把你 Robot.java 的設定搬過來 =====
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = false;

        // 右正、左反（照你原本）
        cfg.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        leftFlywheel.getConfigurator().apply(cfg);

        cfg.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
        rightFlywheel.getConfigurator().apply(cfg);

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = 0.12;
        slot0.kV = 0.119;
        slot0.kA = 0.0;
        slot0.kP = 0.005;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        leftFlywheel.getConfigurator().apply(slot0);
        rightFlywheel.getConfigurator().apply(slot0);

        velocity = leftFlywheel.getVelocity();
        motorVoltage = leftFlywheel.getMotorVoltage();
        statorCurrent = leftFlywheel.getStatorCurrent();
        temp = leftFlywheel.getDeviceTemp();

        SmartDashboard.putNumber("Shooter/TargetRPS", 0.0);
    }

    /** 設定目標轉速 (RPS)，<=0 就停 */
    public void setTargetRps(double rps) {
        if (rps <= 0) {
            leftFlywheel.set(0);
            rightFlywheel.set(0);
        } else {
            leftFlywheel.setControl(m_velReq.withVelocity(rps));
            rightFlywheel.setControl(m_velReq.withVelocity(rps));
        }
        SmartDashboard.putNumber("Shooter/TargetRPS", rps);
    }

    public double getRps() {
        return velocity.getValueAsDouble();
    }

    public void stopFlywheel() {
        setTargetRps(0);
    }
    public Command flyWheelShoot(double rps) {
    // 就像你寫 Feeder 的方式一樣
    return Commands.runOnce(() -> this.setTargetRps(rps), this);
}

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(velocity, motorVoltage, statorCurrent, temp);

        SmartDashboard.putNumber("Shooter/RPS", getRps());
        SmartDashboard.putNumber("Shooter/Voltage", motorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/StatorCurrent", statorCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TempC", temp.getValueAsDouble());
    }
}
