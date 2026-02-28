package frc.robot.commands;

import java.security.DrbgParameters;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class autoRotation extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final LimeLight m_LimeLight;

    // 1. 在這裡定義 PID 控制器 (成員變數)
    // 數值順序是 (kP, kI, kD)
    private final PIDController m_controller = new PIDController(0.01, 0, 0.003501);

    private static final String LIMELIGHT_NAME = "limelight";

    public autoRotation(SwerveSubsystem swerve, LimeLight limelight) {
        this.m_SwerveSubsystem = swerve;
        this.m_LimeLight = limelight;

        // 2. 在構造函數設定容許誤差與目標
        m_controller.setTolerance(1.0); // 誤差 1 度以內視為對準
        m_controller.setSetpoint(0); // 目標是讓 tx (偏移量) 等於 0

        addRequirements(m_SwerveSubsystem, m_LimeLight);
    }

    @Override
    public void initialize() {
        // 3. 每次開始執行指令時，重置 PID 內部狀態（清空積分與微分紀錄）
        m_controller.reset();
    }

    @Override
    public void execute() {
        LimelightHelpers.setLimelightNTDouble(LIMELIGHT_NAME, "pipeline", 0);

        // 4. 計算輸出：傳入目前的 tx 值
        // 它會自動幫你計算出適合的旋轉速度 (rotationSpeed)
        double rotationSpeed = m_controller.calculate(m_LimeLight.getTX());

        // 5. 套用到 Swerve
        SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                .toSwerveModuleStates(new ChassisSpeeds(0, 0, rotationSpeed));

        m_SwerveSubsystem.setModuleStates(moduleStates);
    }

    @Override

    public void end(boolean interrupted) {

        m_SwerveSubsystem.stopModules(); // 確保停止

    }

    @Override

    public boolean isFinished() {

        return false;

    }

}
