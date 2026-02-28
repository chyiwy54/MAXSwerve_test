package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DriveAndAlignCommand extends Command {
    private final SwerveSubsystem drive;
    private final LimeLight vision;

    // 1. 宣告 PID 控制器 (P 值需要根據妳的機器人重量微調)
    private final PIDController distancePID = new PIDController(0.2, 0, 0);
    private final PIDController turnPID = new PIDController(0.02, 0, 0);

    // 設定目標距離 (公尺)
    private final double TARGET_RANGE = 1.54;

    public DriveAndAlignCommand(SwerveSubsystem drive, LimeLight vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);

        // 設定容忍誤差
        distancePID.setTolerance(0.05); // 5公分內視為達標
        turnPID.setTolerance(1.0); // 1度內視為達標
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double tx = vision.getTX();
            double currentDist = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").avgTagDist;

            // --- 關鍵修正：判斷是否已經到達距離目標 ---
            double forwardSpeed;
            if (distancePID.atSetpoint()) {
                // 如果在 0.7m 的 5公分範圍內，前後動力就給 0
                forwardSpeed = 0;
            } else {
                // 否則繼續計算。注意：calculate 的標準邏輯是 (現在值, 目標值)
                // 如果 1.6m -> 0.7m，calculate 通常回傳負值，所以妳加負號變正值前進是正確的
                forwardSpeed = -distancePID.calculate(currentDist, TARGET_RANGE);
                System.out.println("Limelight Dist: " + currentDist);
            }

            // 旋轉部分同樣建議判斷是否已經對準
            double rotationSpeed = turnPID.atSetpoint() ? 0 : turnPID.calculate(tx, 0);

            // 傳送給底盤
            drive.drive(forwardSpeed, 0, rotationSpeed);

        } else {
            drive.drive(0, 0, 0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Align Command Ended. Interrupted: " + interrupted);
        drive.stopModules();
    }

    @Override
public void initialize() {
    System.out.println(">>> Start");
    distancePID.reset(); // 務必重置，確保計算從頭開始
    turnPID.reset();
}

    public boolean isFinished() {
        return false;
    }
}