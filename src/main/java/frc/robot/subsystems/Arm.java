package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// 確保引用的是 REV 的 RelativeEncoder
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Arm extends SubsystemBase {
    private final SparkFlex arm = new SparkFlex(12, MotorType.kBrushless);
    private final PIDController armPid = new PIDController(0.07, 0.0, 0);
    private final RelativeEncoder armEncoder; // 保持這個變數來儲存實體
    private double targetPosition = 0;

    public Arm() {
        // 修改重點：直接從 arm 物件獲取 Encoder
        this.armEncoder = arm.getEncoder();
        // 初始化位置
        this.armEncoder.setPosition(0);
    }

    // 建議加上範圍限制 (軟體限位)，防止手臂轉過頭
    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    public void updateArm() {
        arm.set(armPid.calculate(armEncoder.getPosition(), this.targetPosition));
    }
    
    public double getTargetPosition() {
        return this.targetPosition;
    }

    public double getCurrentPosition() {
        return armEncoder.getPosition();
    }
    
    public Command armMoveToDown() {
        return Commands.runOnce(() -> {this.targetPosition = ControllerConstants.ARM_DOWN; }, this);
    }

    public void stopArm() {
        this.arm.stopMotor();
    }

    @Override
    public void periodic() {
        updateArm();
        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Arm Target Position", targetPosition);
    }
}