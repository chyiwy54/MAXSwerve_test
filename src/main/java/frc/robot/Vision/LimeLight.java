package frc.robot.Vision;
// 套件路徑：Vision 相關程式放這裡（Java package）

import com.ctre.phoenix6.hardware.Pigeon2;
// Pigeon2 陀螺儀硬體（Phoenix6），可拿 yaw/pitch/roll 與角速度

import frc.robot.Constants;
// 你的常數總表（這支檔案目前沒有用到 Constants 本體，留著也不影響編譯）

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// AprilTag 場地配置（此檔未使用，可刪）

import edu.wpi.first.apriltag.AprilTagFields;
// WPILib 內建各賽季 AprilTag 配置（此檔未使用，可刪）

import edu.wpi.first.math.VecBuilder;
// 用來建立 (xStd,yStd,thetaStd) 的向量，給 PoseEstimator 使用
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
// 單位轉換，例如 degreesToRadians

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// SubsystemBase：讓這個類別成為 Command-based 子系統，能有 periodic()

import frc.robot.Constants.FieldConstants;
// 場地長寬等常數，用於檢查 pose 是否在場地內

import frc.robot.Constants.LimelightConstants;
// Limelight 相關常數（例如 MAX_GYRO_RATE 旋轉太快就不信 vision）

import frc.robot.subsystems.Swerve.SwerveSubsystem;
// Swerve 底盤子系統（要把 vision measurement 餵進去）

public class LimeLight extends SubsystemBase {
    // LimeLight 子系統：負責讀 Limelight → 過濾 → 餵給底盤 PoseEstimator / Odometry

    private final SwerveSubsystem drive;
    // 底盤 subsystem 參考：用來 addVisionMeasurement() / resetOdometry() / 拿 gyro 等

    private final String limelightName;
    // Limelight 名稱（NetworkTables table name），例如 "limelight"、"limelight-front"

    private final Pigeon2 gyro;
    // Pigeon2 gyro：提供機器人姿態/角速度給 MegaTag2 使用

    private int tagId = -1;
    // 當前看到的 AprilTag ID；-1 表示目前沒有有效 tag

    public LimeLight(SwerveSubsystem drive, String limelightName) {
        // 建構子：外部傳入 drive 與 limelightName（依賴注入）

        this.drive = drive;
        // 把傳入的 drive 存起來

        this.limelightName = limelightName;
        // 把傳入的 Limelight 名稱存起來

        this.gyro = drive.getPigeon2();
        // 從 drive 取得 Pigeon2（表示 gyro 在底盤那邊初始化/管理）
    }

    @Override
    public void periodic() {
        // 每個迴圈（通常 20ms）執行一次：更新 Limelight orientation、取 MT2、過濾、餵 pose estimator
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();

        // 2. 處理角度偏移：如果是在紅方，角度要旋轉 180 度，才能符合 wpiBlue 座標系
        double yawOffset = (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red)
                ? 180.0
                : 0.0;

        double correctedYaw = this.gyro.getYaw().getValueAsDouble() + yawOffset;

        LimelightHelpers.SetRobotOrientation(
                // 把機器人 IMU 姿態送給 Limelight（MegaTag2 融合定位必做）

                this.limelightName,
                // 第 1 個參數：要送到哪一顆 limelight（table name）

                correctedYaw, // 使用修正後的角度！ 第 2 個參數：Yaw（偏航角，機器人朝向），單位：度

                this.gyro.getAngularVelocityZWorld().getValueAsDouble(),
                // 第 3 個參數：YawRate / Z 軸角速度（旋轉速度），單位：度/秒

                this.gyro.getPitch().getValueAsDouble(),
                // 第 4 個參數：Pitch（前後仰角），單位：度

                this.gyro.getAngularVelocityYWorld().getValueAsDouble(),
                // 第 5 個參數：PitchRate / Y 軸角速度（前後傾斜變化速度），單位：度/秒

                this.gyro.getRoll().getValueAsDouble(),
                // 第 6 個參數：Roll（左右側傾角），單位：度

                this.gyro.getAngularVelocityXWorld().getValueAsDouble()
        // 第 7 個參數：RollRate / X 軸角速度（左右傾斜變化速度），單位：度/秒
        );
        // SetRobotOrientation 呼叫結束

        // ---------------------------------------------------------
        // 取得 MegaTag2 的 Pose Estimate（WPILib 標準：永遠以藍方原點 wpiblue）
        // ---------------------------------------------------------
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        // 從 Limelight 取得 MegaTag2 的 pose（含 pose、tagCount、avgTagDist、timestampSeconds）

        if (mt2 == null) {
            // 如果 Limelight 沒回傳資料（沒連上、或 pipeline 沒輸出）
            return;
            // 直接跳出，不做後續融合
        }

        tagId = (int) LimelightHelpers.getFiducialID(limelightName);
        // 讀取目前主要看到的 AprilTag ID（若看到多顆，通常是主要那顆）
        // 轉成 int 存到 tagId，給其他地方查詢

        // ---------------------------------------------------------
        // Filters：不符合條件的 vision data 直接丟棄
        // ---------------------------------------------------------
        if (mt2.tagCount == 0) {
            // 若這一幀完全沒有 tag
            return;
            // 不做融合（避免亂飄）
        }

        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > LimelightConstants.MAX_GYRO_RATE) {
            // 如果機器人旋轉太快（Z 軸角速度超過門檻）
            // 旋轉太快時影像可能模糊/殘影，Tag pose 會不穩

            return;
            // 丟掉這一幀的 vision pose，避免把 estimator 拉歪
        }

        if (mt2.pose.getX() < 0
                || mt2.pose.getX() > FieldConstants.fieldLength
                || mt2.pose.getY() < 0
                || mt2.pose.getY() > FieldConstants.fieldWidth) {
            // 若 Limelight 算出的機器人位置超出場地範圍（通常代表解算錯或座標系混用）
            return;
            // 不融合這筆資料
        }

        // ---------------------------------------------------------
        // 計算 Measurement Standard Deviations（信任程度）
        // PoseEstimator 會依 std 來決定 vision 對結果影響有多大
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
    // periodic() 結束
    /**
     * 取得機器人相對於目前目標 Tag 的位姿 (Target Space)。
     * 在斜角對準時，這個 X 座標就是妳要平移的誤差，Yaw 就是妳要旋轉的誤差。
     */
    public Pose3d getRobotPose_TargetSpace() {
        // 取得相對於標籤的 6D Pose (x, y, z, rx, ry, rz)
        double[] poseArray = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (poseArray.length < 6)
            return new Pose3d();

        // 注意：Limelight 的 RobotSpace 和 TargetSpace 座標定義要對照文件
        return LimelightHelpers.toPose3D(poseArray);
    }

    public int getTagId() {
        // 外部要查目前看到哪個 tag ID

        return tagId;
        // 回傳最後一次更新的 tagId（可能是 -1）
    }

    /**
     * 取得目標物相對於攝影機中線的水平偏移角度 (tx)。
     * * @return 水平偏移角度（單位：度）。若無目標則回傳 0.0。
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public boolean resetPoseToVision() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();

        // 2. 處理角度偏移：如果是在紅方，角度要旋轉 180 度，才能符合 wpiBlue 座標系
        double yawOffset = (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red)
                ? 180.0
                : 0.0;

        double correctedYaw = this.gyro.getYaw().getValueAsDouble() + yawOffset;
        // 以 vision pose 強制重設 odometry（通常用在開場或定位飄掉時）

        LimelightHelpers.SetRobotOrientation(
                // 重設前再送一次 IMU 姿態，確保 Limelight 使用最新 gyro

                this.limelightName,
                // Limelight 名稱

                correctedYaw, // 使用修正後的角度！ // yaw（度）

                this.gyro.getAngularVelocityZWorld().getValueAsDouble(),
                // yaw rate（度/秒）

                this.gyro.getPitch().getValueAsDouble(),
                // pitch（度）

                this.gyro.getAngularVelocityYWorld().getValueAsDouble(),
                // pitch rate（度/秒）

                this.gyro.getRoll().getValueAsDouble(),
                // roll（度）

                this.gyro.getAngularVelocityXWorld().getValueAsDouble()
        // roll rate（度/秒）
        );
        // orientation 更新結束

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        // 再抓一次 MegaTag2 pose（用 wpiblue，符合 WPILib 座標系建議）

        if (mt2 == null || mt2.tagCount == 0) {
            // 若沒有資料或沒有看到 tag
            return false;
            // 回傳 false 表示重設失敗
        }

        if (mt2.avgTagDist > 4.0) {
            // 若平均距離太遠（>4m），vision 誤差可能太大
            return false;
            // 避免把 odometry 重設到錯的位置
        }

        if (mt2.pose.getX() < 0
                || mt2.pose.getX() > FieldConstants.fieldLength
                || mt2.pose.getY() < 0
                || mt2.pose.getY() > FieldConstants.fieldWidth) {
            // 若 vision pose 不在場地內
            return false;
            // 不重設
        }

        drive.resetOdometry(mt2.pose);
        // 強制把底盤 odometry / pose estimator 的 pose 設為 vision pose（瞬移）

       System.out.println("Odometry reset to vision pose: " + mt2.pose.toString());
        // 印出資訊到 console，方便你確認重設時的 pose

        return true;
        // 回傳 true 表示重設成功
    }
    // resetPoseToVision() 結束

}
// class 結束
