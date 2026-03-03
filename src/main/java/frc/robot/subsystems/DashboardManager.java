package frc.robot.subsystems; // 請替換成你們專案實際的 package 路徑

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class DashboardManager {

    public void updateGameDataDashboard() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        boolean isHubActive = true;
        double countdownToNextShift = 0.0;

        // 1. 計算下一個 Shift 的倒數時間 (matchTime 是遞減的剩餘時間)
        if (matchTime > 130) {
            countdownToNextShift = matchTime - 130; // 距離 Shift 1 開始
        } else if (matchTime > 105) {
            countdownToNextShift = matchTime - 105; // 距離 Shift 2 開始
        } else if (matchTime > 80) {
            countdownToNextShift = matchTime - 80;  // 距離 Shift 3 開始
        } else if (matchTime > 55) {
            countdownToNextShift = matchTime - 55;  // 距離 Shift 4 開始
        } else if (matchTime > 30) {
            countdownToNextShift = matchTime - 30;  // 距離 End Game 開始 (Hub 全開)
        } else if (matchTime > 0) {
            countdownToNextShift = matchTime;       // 距離比賽結束
        } else {
            countdownToNextShift = 0.0;
        }

        // 2. 計算目前 Hub 是否為開啟狀態 (依據官方 2026 Game Data 邏輯)
        if (alliance.isPresent() && DriverStation.isTeleopEnabled() && !gameData.isEmpty()) {
            boolean redInactiveFirst = gameData.charAt(0) == 'R';
            boolean isRedAlliance = alliance.get() == Alliance.Red;

            // Shift 1 狀態：如果紅隊先失效，那藍隊在 Shift 1 就是開啟的
            boolean shift1Active = isRedAlliance ? !redInactiveFirst : redInactiveFirst;

            if (matchTime > 130) {
                isHubActive = true; // Transition period
            } else if (matchTime > 105) {
                isHubActive = shift1Active; // Shift 1
            } else if (matchTime > 80) {
                isHubActive = !shift1Active; // Shift 2
            } else if (matchTime > 55) {
                isHubActive = shift1Active; // Shift 3
            } else if (matchTime > 30) {
                isHubActive = !shift1Active; // Shift 4
            } else {
                isHubActive = true; // End game, Hub 永遠開啟
            }
        } else {
            // 如果沒有資料、非 Teleop、或未連上 FMS，安全起見預設為開啟
            isHubActive = true;
        }

        // 3. 推送到 SmartDashboard/NetworkTables
        SmartDashboard.putBoolean("Hub Active Status", isHubActive);
        
        // 將倒數時間四捨五入到小數點後一位，避免儀表板數字跳動太快影響閱讀
        double roundedCountdown = Math.round(countdownToNextShift * 10.0) / 10.0;
        SmartDashboard.putNumber("Shift Countdown", roundedCountdown);
    }
}