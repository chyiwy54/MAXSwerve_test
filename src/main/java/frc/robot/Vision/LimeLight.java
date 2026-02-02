package frc.robot.Vision;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class LimeLight extends SubsystemBase {

    private final SwerveSubsystem drive;
    private final String limelightName;

    private final Pigeon2 gyro;

    private int tagId = -1;
    public LimeLight(
            SwerveSubsystem drive,
            String limelightName) {
        this.drive = drive;
        this.limelightName = limelightName;
        this.gyro = drive.getPigeon2();
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(
                this.limelightName,
                this.gyro.getYaw().getValueAsDouble(),
                this.gyro.getAngularVelocityZWorld().getValueAsDouble(),
                this.gyro.getPitch().getValueAsDouble(),
                this.gyro.getAngularVelocityYWorld().getValueAsDouble(),
                this.gyro.getRoll().getValueAsDouble(),
                this.gyro.getAngularVelocityXWorld().getValueAsDouble());

        // MegaTag2 result

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null)
            return;

        tagId = (int) LimelightHelpers.getFiducialID(limelightName);

        // Filterts

        if (mt2.tagCount == 0)
            return;

        // 機器人旋轉太快時 (大於 maxYawRate度/秒)，視覺會有殘影，不使用數據
        // (假設 drive.getGyroYawRate() 回傳 deg/s)
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > LimelightConstants.MAX_GYRO_RATE)
            return;

        // 檢查座標是否跑出場地外 (X: 0~16.54m, Y: 0~8.21m)
        if (mt2.pose.getX() < 0 || mt2.pose.getX() > FieldConstants.fieldLength ||
                mt2.pose.getY() < 0 || mt2.pose.getY() > FieldConstants.fieldWidth)
            return;

        // ---------------------------------------------------------
        // 4. 計算標準差 (Trust Level)
        // ---------------------------------------------------------
        double xyStds;
        double degStds;
        double avgDist = mt2.avgTagDist;

            if (mt2.tagCount >= 2) {
                // 多 Tag：非常信任
                xyStds = 0.5;
                degStds = 6.0;
            } else {
                // 單 Tag：信任度隨距離遞減 (距離越遠，標準差越大)
                // 這裡使用距離的平方來快速降低遠距離的權重
                xyStds = 1.0 * (avgDist * avgDist);
                degStds = 999.0; // 單 Tag 完全不信任 MT2 算出的角度，只用它的 X/Y
            }

        // ---------------------------------------------------------
        // 5. 送入 Drive Subsystem
        // ---------------------------------------------------------
        // 這裡需要你的 Drive 支援接收標準差 (Vector<N3>)
        drive.addVisionMeasurement(
                mt2.pose, // 視覺算出的 Pose2d
                mt2.timestampSeconds, // 這是正確的拍攝時間 (Latency Compensated)
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
    }

    // 提供給外部使用的 Getter
    public int getTagId() {
        return tagId;
    }
    public boolean resetPoseToVision() {
        // 1. 確保 Limelight 收到最新的 Gyro 數據 (雖然 periodic 會跑，但為了保險再送一次)
        LimelightHelpers.SetRobotOrientation(
                this.limelightName,
                this.gyro.getYaw().getValueAsDouble(),
                this.gyro.getAngularVelocityZWorld().getValueAsDouble(),
                this.gyro.getPitch().getValueAsDouble(),
                this.gyro.getAngularVelocityYWorld().getValueAsDouble(),
                this.gyro.getRoll().getValueAsDouble(),
                this.gyro.getAngularVelocityXWorld().getValueAsDouble());

        // 2. 取得 Pose Estimate (建議初始重設使用 MegaTag1 或 MegaTag2 皆可，這裡沿用你的 MT2)
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        // 3. 基本檢查
        if (mt2 == null || mt2.tagCount == 0) {
            return false;
        }

        // 4. 安全過濾：初始重設時，我們希望數據非常準確
        // 如果距離 Tag 太遠 (例如大於 4 公尺)，誤差會變大，建議不要重設
        if (mt2.avgTagDist > 4.0) {
            return false;
        }

        // 檢查座標是否在場地內
        if (mt2.pose.getX() < 0 || mt2.pose.getX() > FieldConstants.fieldLength ||
            mt2.pose.getY() < 0 || mt2.pose.getY() > FieldConstants.fieldWidth) {
            return false;
        }

        // 5. 執行重設
        // 這裡呼叫 drive 的 resetOdometry，這會強制把機器人座標瞬移到視覺看到的地方
        drive.resetOdometry(mt2.pose);
        
        // (選用) 如果你想在 Dashboard 上看到重設成功的訊息
        System.out.println("Odometry reset to vision pose: " + mt2.pose.toString());
        
        return true;
    }
}