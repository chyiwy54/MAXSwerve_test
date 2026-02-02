package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;

public class Driver extends XboxController {
    // 修改點：加入倍率 (Rate Multiplier)
    // 3.0 代表約 0.33 秒煞停。如果不乘，機器人會滑行 1.0 秒。
    private static final double kRateMultiplier = 3.0; 

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    public Driver(int port) { // 建議加入 port 參數增加彈性，或者維持 super(0)
        super(port);
        // 初始化時乘上倍率
        this.xLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond * kRateMultiplier);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond * kRateMultiplier);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularSpeed * kRateMultiplier);
    }

    public double getXLimiter() {
        // Deadband
        double input = MathUtil.applyDeadband(this.getLeftY(), 0.05);
        // 計算並回傳 "公尺/秒"
        return this.xLimiter.calculate(-input * DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public double getYLimiter() {
        double input = MathUtil.applyDeadband(this.getLeftX(), 0.05);
        return this.yLimiter.calculate(-input * DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public double getTurningLimiter() {
        double input = MathUtil.applyDeadband(this.getRightX(), 0.05);
        // 注意：旋轉應該乘上 kMaxAngularSpeed (Radians/Sec)
        return this.turningLimiter.calculate(-input * DriveConstants.kMaxAngularSpeed);
    }
}