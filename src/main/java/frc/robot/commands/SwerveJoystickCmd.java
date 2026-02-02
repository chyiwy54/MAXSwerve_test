package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.Swerve.AutoAlign;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Driver driver;
    private final AutoAlign autoAlign; // 新增 AutoAlign 物件

    // 新增旋轉用的 PID 控制器
    // kP, kI, kD 需要根據你的機器人調整 (建議從 kP = 3.0 開始測試)
    private final PIDController turnPID = new PIDController(4.0, 0.0, 0.0);

    public SwerveJoystickCmd(
            SwerveSubsystem swerveSubsystem,
            Driver driver) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.driver = driver;
        this.autoAlign = new AutoAlign(swerveSubsystem); // 初始化 AutoAlign

        // 設定 PID 為連續輸入 (因為 -PI 和 PI 是同一個點)
        // 範圍是 -PI 到 PI (Radians)
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // 1. 取得 X 和 Y 的速度 (這部分保持不變)
        double xSpeed = driver.getXLimiter(); 
        double ySpeed = driver.getYLimiter(); 
        
        // 2. 處理旋轉速度 (Rot)
        double rotPercent;

        // --- 關鍵修改開始 ---
        // 檢查 Driver 的 LB (Left Bumper) 是否按下
        if (driver.getLeftBumper()) {
            // A. 自動對準模式
            
            // 取得目標角度 (來自 AutoAlign)
            double targetAngleRadians = autoAlign.AutoTurn().getRadians();
            
            // 取得目前機器人角度
            double currentAngleRadians = swerveSubsystem.getRotation2d().getRadians();

            // 使用 PID 計算需要的角速度 (Radians Per Second)
            double calcRotSpeed = turnPID.calculate(currentAngleRadians, targetAngleRadians);

            // 將角速度轉換為百分比 (-1.0 ~ 1.0)
            rotPercent = calcRotSpeed / DriveConstants.kMaxAngularSpeed;

            // 加上 Clamp 以防數值超過範圍
            rotPercent = MathUtil.clamp(rotPercent, -1.0, 1.0);

        } else {
            // B. 手動模式 (原本的邏輯)
            double rotSpeed = driver.getTurningLimiter();
            rotPercent = rotSpeed / DriveConstants.kMaxAngularSpeed;
        }
        // --- 關鍵修改結束 ---

        // 3. 處理 X 和 Y 的百分比轉換
        double xPercent = xSpeed / DriveConstants.kMaxSpeedMetersPerSecond;
        double yPercent = ySpeed / DriveConstants.kMaxSpeedMetersPerSecond;

        // 4. 傳送給 Subsystem
        swerveSubsystem.drive(
            xPercent, 
            yPercent, 
            rotPercent // 這裡現在會根據是否按下 LB 自動切換
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}