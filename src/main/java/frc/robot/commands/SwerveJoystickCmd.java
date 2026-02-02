package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Driver driver; // 使用你自定義的 Driver 類別

    public SwerveJoystickCmd(
            SwerveSubsystem swerveSubsystem,
            Driver driver) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.driver = driver;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // 1. 從 Driver 類別取得已經處理過 SlewRate 和 Deadband 的 "實際速度 (m/s)"
        double xSpeed = driver.getXLimiter(); 
        double ySpeed = driver.getYLimiter(); 
        double rotSpeed = driver.getTurningLimiter();

        // 2. 單位轉換 (重要！)
        // 你的 Driver 類別已經乘上了 MaxSpeed (例如回傳 6.5 m/s)。
        // 但是 SwerveSubsystem 的 drive() 方法通常會自己再乘一次 MaxSpeed。
        // 如果我們不除回去，機器人會試圖跑出 6.5 * 6.5 = 42.25 m/s 的速度 (會出錯)。
        
        double xPercent = xSpeed / DriveConstants.kMaxSpeedMetersPerSecond;
        double yPercent = ySpeed / DriveConstants.kMaxSpeedMetersPerSecond;
        double rotPercent = rotSpeed / DriveConstants.kMaxAngularSpeed;

        // 3. 傳送給 Subsystem
        swerveSubsystem.drive(
            xPercent, 
            yPercent, 
            rotPercent
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