package frc.robot.subsystems.Swerve;
// 這個類別所在的 package（Swerve 子系統底下的工具/邏輯）

import edu.wpi.first.math.geometry.Pose2d;
// Pose2d：機器人在場地上的位置 (x,y) + 朝向 rotation

import edu.wpi.first.math.geometry.Rotation2d;
// Rotation2d：2D 平面旋轉角度（用弧度表示，但你通常用 degrees 觀念理解）

import edu.wpi.first.math.geometry.Transform2d;
// Transform2d：2D 位移+旋轉的「變換」（這份程式目前沒有用到，可刪但不影響）

import edu.wpi.first.math.geometry.Translation2d;
// Translation2d：2D 平面的位置/向量 (x,y)

import frc.robot.Constants.FieldConstants.siteConstants;
// 你自訂的場地常數（裡面有各種目標點、tag 點、對準點等）

import frc.robot.utils.AllianceFlipUtil;
// 你自訂的聯盟翻轉工具（用來把「藍方定義的點」翻到紅方對稱位置）

public class AutoAlign {
    // AutoAlign：自動對準的工具類別（根據機器人目前 pose，算出要朝向哪個目標）

    private final SwerveSubsystem drive;
    // 保存底盤子系統參考，用來取得機器人目前 pose（drive.getPose()）

    public AutoAlign(SwerveSubsystem drive) {
        // 建構子：把底盤子系統注入進來（dependency injection）
        this.drive = drive;
        // 將傳進來的 drive 存成成員變數，之後 AutoTurn() 會用到
    }
    


    public Rotation2d AutoTurn() {
        // AutoTurn：計算「機器人應該轉到的角度」
        // 回傳 Rotation2d（目標朝向角）

        Pose2d robotPose = drive.getPose();
        // 取得目前機器人的 pose（通常是 PoseEstimator/odometry 的結果）
        // ⚠️ 正確做法：robotPose 應該永遠是 WPILib 的 wpiblue 座標系（藍原點）


        // 記得將 Translation3d 轉為 Translation2d (因為底盤旋轉只需要 X, Y)
        Translation2d target = AllianceFlipUtil.apply(
            siteConstants.getTargetPoint(0).toTranslation2d()
        );
        // 1) siteConstants.getTargetPoint(0) 取出「藍方定義」的目標點（可能原本是 Translation3d）
        // 2) toTranslation2d() 把 3D 轉成 2D（忽略高度 Z）
        // 3) AllianceFlipUtil.apply(...) 依照目前聯盟顏色決定：
        //    - 藍方：回傳原點
        //    - 紅方：回傳「對稱翻轉」後的目標點
        // ✅ 這裡只 flip 目標點，不 flip robotPose，這樣不會翻兩次

        Translation2d vectorToTarget = target.minus(robotPose.getTranslation());
        // 計算「從機器人位置指向目標點」的向量
        // target - robotTranslation = (dx, dy)
        // 這個向量表示：目標在機器人相對於場地的哪個方向

        return vectorToTarget.getAngle();
        // 回傳這個向量的角度（也就是機器人要面向目標的場地方位角）
        // 注意：這是「場地座標角度」，若你底盤控制需要的是「相對角度」或「轉多少」要再用 PID 比較目前 heading
    }
}
// 類別結束
