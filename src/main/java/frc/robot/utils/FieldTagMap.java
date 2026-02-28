package frc.robot.utils;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * 2026 Rebuilt 賽季場地 AprilTag 座標地圖工具類別
 */
public class FieldTagMap {

    // 儲存所有標籤 ID 與其 3D 座標的 Map
    public static final Map<Integer, Pose3d> tagMap = new HashMap<>();

    // 定義左側 Trench（壕溝/特定設施）對應的 AprilTag ID
    private static final int[] LEFT_TRENCH_IDS = { 22, 23 };

    // 定義右側 Trench 對應的 AprilTag ID
    private static final int[] RIGHT_TRENCH_IDS = { 17, 28 };

    // 用於計算偏移的半寬度（單位：公尺），1.668 可能是某個設施的全寬
    private static final double HALF_WIDTH = 1.668 / 2.0;

    // 靜態程式區塊，類別載入時立即執行 loadTags()
    static { loadTags(); }

    /**
     * 從 WPILib 載入場地預設的 AprilTag 配置
     */
    private static void loadTags() {
        // 載入 2026 賽季的預設場地佈局檔案
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        // 遍歷所有標籤並存入 tagMap 中，方便後續快速查詢
        for (AprilTag tag : layout.getTags()) {
            tagMap.put(tag.ID, tag.pose);
        }
    }

    /**
     * 根據標籤 ID 取得 3D 空間座標 (X, Y, Z + 旋轉)
     */
    public static Pose3d getPose3d(int id) {
        return tagMap.get(id);
    }

    /**
     * 根據標籤 ID 取得 2D 平面座標 (X, Y + 偏航角)
     */
    public static Pose2d getPose2d(int id) {
        Pose3d pose3d = tagMap.get(id);
        if (pose3d == null) return null; // 如果找不到 ID 則回傳 null
        return pose3d.toPose2d(); // 將 3D 座標投影到 2D 平面
    }

    /**
     * 檢查地圖中是否存在該 ID 的標籤
     */
    public static boolean hasTag(int id) {
        return tagMap.containsKey(id);
    }

    /**
     * 取得左側 Trench 的中心參考點座標
     */
    public static Pose2d getLeftTrenchPose() {
        return calculateAveragePose(LEFT_TRENCH_IDS);
    }

    /**
     * 取得右側 Trench 的中心參考點座標
     */
    public static Pose2d getRightTrenchPose() {
        return calculateAveragePose(RIGHT_TRENCH_IDS);
    }

    /**
     * 計算多個標籤 ID 的幾何中心座標與平均角度
     */
    private static Pose2d calculateAveragePose(int[] ids) {
        Pose2d pose1 = FieldTagMap.getPose2d(ids[0]);
        Pose2d pose2 = FieldTagMap.getPose2d(ids[1]);

        // 計算 X 與 Y 座標的算術平均值
        double avgX = (pose1.getX() + pose2.getX()) / 2.0;
        double avgY = (pose1.getY() + pose2.getY()) / 2.0;

        // 利用三角函數處理角度平均，避免直接平均角度造成的邏輯衝突（如 350° 和 10°）
        double avgCos = pose1.getRotation().getCos() + pose2.getRotation().getCos();
        double avgSin = pose1.getRotation().getSin() + pose2.getRotation().getSin();

        // 建立平均後的旋轉角度
        Rotation2d avgRot = new Rotation2d(avgCos, avgSin);

        return new Pose2d(avgX, avgY, avgRot);
    }

    /**
     * 取得左側 Trench 前方的兩個偏移對齊點 (用於雙路徑或避開障礙)
     */
    public static Pose2d[] getLeftTrenchPoses() {
        Pose2d center = calculateAveragePose(LEFT_TRENCH_IDS);
        return applyXOffset(center);
    }

    /**
     * 取得右側 Trench 前方的兩個偏移對齊點
     */
    public static Pose2d[] getRightTrenchPoses() {
        Pose2d center = calculateAveragePose(RIGHT_TRENCH_IDS);
        return applyXOffset(center);
    }

    /**
     * 針對傳入的座標，在 X 軸方向往左與往右偏移 HALF_WIDTH 的距離
     */
    private static Pose2d[] applyXOffset(Pose2d center) {
        // 中心點 X 減去半寬
        Pose2d poseMinus = new Pose2d(
            center.getX() - HALF_WIDTH, 
            center.getY(), 
            center.getRotation()
        );
        // 中心點 X 加上半寬
        Pose2d posePlus = new Pose2d(
            center.getX() + HALF_WIDTH, 
            center.getY(), 
            center.getRotation()
        );

        return new Pose2d[] { poseMinus, posePlus };
    }
}