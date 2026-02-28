package frc.robot.commands;
// 這個類別所在的 package（command 層）

import edu.wpi.first.math.MathUtil;
// 數學工具（這裡用來 clamp 數值在範圍內）

import edu.wpi.first.math.controller.PIDController;
// WPILib PID 控制器（用來控制旋轉角度）
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
// Command-based 架構的 Command 類別

import frc.robot.Constants.DriveConstants;
// 底盤相關常數，例如最大速度、最大角速度
import frc.robot.Robot;
import frc.robot.joystick.Driver;
// 你自訂的 Driver 控制器封裝（XboxController）

import frc.robot.subsystems.Swerve.AutoAlign;
// 自動對準邏輯（計算應該朝向哪個角度）

import frc.robot.subsystems.Swerve.SwerveSubsystem;
// Swerve 底盤子系統

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    // 底盤 subsystem，用來控制機器人運動

    private final Driver driver;
    // Driver 控制器

    private final AutoAlign autoAlign;
    // AutoAlign 物件，用來計算自動對準角度

    // 新增旋轉用的 PID 控制器
    // kP, kI, kD 需要根據你的機器人調整
    private final PIDController turnPID = new PIDController(2.0, 0.0, 0.0);
    // 這個 PID 會控制「角度誤差」→ 輸出旋轉速度

    public SwerveJoystickCmd(
            SwerveSubsystem swerveSubsystem,
            Driver driver) {

        this.swerveSubsystem = swerveSubsystem;
        // 保存底盤 subsystem

        this.driver = driver;
        // 保存控制器

        this.autoAlign = new AutoAlign(swerveSubsystem);
        // 初始化 AutoAlign（內部會用 drive.getPose()）

        // 設定 PID 為連續輸入
        // 因為角度 -PI 和 PI 是同一個方向
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
        // 告訴 scheduler：這個 command 需要控制 swerveSubsystem
    }

    @Override
    public void execute() {

        // 1. 取得 X 和 Y 的速度 (driver 搖桿輸入)
        double xSpeed = driver.getXLimiter();
        double ySpeed = driver.getYLimiter();
        // 這兩個通常已經經過 slew rate limiter，避免加速太猛

        // 2. 處理旋轉速度 (Rot)
        double rotPercent;
         // --- 關鍵修改開始 ---
        // 檢查 Driver 的 LB 是否按下
        if (driver.getLeftBumper()) {

            // A. 自動對準模式
            double targetAngleRadians = autoAlign.AutoTurn().getRadians();
            double currentAngleRadians = swerveSubsystem.getRotation2d().getRadians();

            double calcRotSpeed = turnPID.calculate(currentAngleRadians, targetAngleRadians);

            // --- 修改部分 ---
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
                // 如果在藍方方向是反的，就在這裡加負號修正
                calcRotSpeed = -calcRotSpeed;
            }
            // ----------------

            rotPercent = calcRotSpeed / DriveConstants.kMaxAngularSpeed;
            // kMaxAngularSpeed 通常是 rad/s%F%

            // 防止超過範圍
            rotPercent = MathUtil.clamp(rotPercent, -1.0, 1.0);

        } else {

            // B. 手動模式
            double rotSpeed = driver.getTurningLimiter();
            // 搖桿旋轉輸入（已經過 limiter）

            rotPercent = rotSpeed / DriveConstants.kMaxAngularSpeed;
        }
        // --- 關鍵修改結束 ---
        // ------------------------------------------------

        // 3. 處理 X 和 Y 的百分比轉換
        double xPercent = xSpeed / DriveConstants.kMaxSpeedMetersPerSecond;
        double yPercent = ySpeed / DriveConstants.kMaxSpeedMetersPerSecond;
        // 把 m/s 轉成 -1~1 百分比

        // 4. 傳送給 Subsystem
        swerveSubsystem.drive(
                xPercent,
                yPercent,
                rotPercent);
        // drive() 內部會轉換為 chassis speeds → module states
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        // command 結束時停止底盤
    }

    @Override
    public boolean isFinished() {
        return false;
        // 永遠不自動結束（手動駕駛 command）
    }
}
