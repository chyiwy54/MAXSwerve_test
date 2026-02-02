package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);
    private final SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.kMaxSpeedMetersPerSecond);

    public Driver() {
        super(0);
    }

    public double getXLimiter() {
        double speed = MathUtil.applyDeadband(this.getLeftY(),0.05);
        return this.xLimiter.calculate(-speed * DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public double getYLimiter() {
        double speed = MathUtil.applyDeadband(this.getLeftX(),0.05);
        return this.yLimiter.calculate(-speed *  DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public double getTurningLimiter() {
        double speed = MathUtil.applyDeadband(this.getRightX(),0.05);
        return this.turningLimiter.calculate(-speed * DriveConstants.kMaxAngularSpeed);
    }
}