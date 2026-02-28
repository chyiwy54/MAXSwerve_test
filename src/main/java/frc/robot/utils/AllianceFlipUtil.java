package frc.robot.utils; // 定義套件路徑，存放於工具類別資料夾

import edu.wpi.first.math.geometry.Pose2d; // 引入 WPILib 的位姿類別 (包含 X, Y, 角度)
import edu.wpi.first.math.geometry.Rotation2d; // 引入角度處理類別
import edu.wpi.first.math.geometry.Translation2d; // 引入座標平移類別 (僅 X, Y)
import edu.wpi.first.wpilibj.DriverStation; // 引入驅動站類別，用來獲取比賽狀態
import edu.wpi.first.wpilibj.DriverStation.Alliance; // 引入聯盟顏色（紅/藍）的列舉型別
import frc.robot.Constants.FieldConstants; // 引入場地常數（如場地總長度）

/**
 * 聯盟翻轉工具類別
 * 用於將「藍色聯盟視角」的座標，在必要時轉換為「紅色聯盟視角」
 */
public class AllianceFlipUtil {

    /**
     * 對 Translation2d (點座標) 進行翻轉
     * @param translation 原始座標 (通常以藍色方為基準)
     * @return 翻轉後的座標
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) { // 如果判斷當前是紅色聯盟（需要翻轉）
            return new Translation2d(
                    FieldConstants.fieldLength - translation.getX(), // X 軸翻轉：場地總長減去原始 X
                    translation.getY()); // Y 軸保持不變 (假設場地是左右鏡像對稱)
        }
        return translation; // 如果是藍色聯盟，則直接回傳原始座標
    }

    /**
     * 對 Pose2d (位姿) 進行翻轉，包含座標與朝向角度
     * @param pose 原始位姿
     * @return 翻轉後的位姿
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) { // 如果判斷需要翻轉
            return new Pose2d(
                    FieldConstants.fieldLength - pose.getX(), // X 軸進行鏡像翻轉
                    pose.getY(), // Y 軸座標保持不變
                    // 角度翻轉邏輯：將向量的 X 分量取負號，Y 分量不變，達成 180度 - theta 的效果
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()) 
            );
        }
        return pose; // 如果不需翻轉，直接回傳
    }

    /**
     * 判斷目前是否需要進行翻轉
     * 邏輯：當 FMS 分派我們為「紅色聯盟」時，回傳 true
     * @return 是否翻轉的布林值
     */
    public static boolean shouldFlip() {
        var alliance = DriverStation.getAlliance(); // 從驅動站獲取目前的聯盟資訊
        if (alliance.isPresent()) { // 檢查是否已連線並取得聯盟資料
            return alliance.get() == Alliance.Red; // 如果是紅色聯盟，則回傳 true
        }
        return false; // 若未連線或為藍色聯盟，預設不翻轉 (回傳 false)
    }
}