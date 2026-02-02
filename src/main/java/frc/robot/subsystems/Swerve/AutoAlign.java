package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants.siteConstants;
import frc.robot.utils.AllianceFlipUtil;

public class AutoAlign {
    private final SwerveSubsystem drive;

    public AutoAlign(SwerveSubsystem drive) {
        this.drive = drive;
    }

    public Rotation2d AutoTurn() {
        Pose2d robotPose = drive.getPose();

        Pose2d turretPose = robotPose;

        Translation2d target = AllianceFlipUtil.apply(siteConstants.topCenterPoint.toTranslation2d());

        // 4. 計算從砲塔指向目標的向量
        Translation2d vectorToTarget = target.minus(turretPose.getTranslation());

        // 6. 計算目標的場地角度 (Field-Relative Angle)
        Rotation2d targetFieldAngle = vectorToTarget.getAngle();

        return targetFieldAngle;
    }
}
