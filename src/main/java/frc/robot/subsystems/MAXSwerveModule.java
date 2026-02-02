package frc.robot.subsystems; // 請確認這是否是你想要的 package 路徑

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder; // 用這個取代 CANcoder

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// 引用原本的 Configs
import frc.robot.Configs;

public class MAXSwerveModule extends SubsystemBase {

  private final SparkFlex driveMotor;
  private final SparkMax steerMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder; // 取代 CANcoder，因為你的編碼器接在 Spark 上

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController steerController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public MAXSwerveModule(
      int driveMotorID,
      int steerMotorID,
      double chassisAngularOffset) {
    this.driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
    this.steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);

    this.driveController = this.driveMotor.getClosedLoopController();
    this.steerController = this.steerMotor.getClosedLoopController();

    this.driveEncoder = this.driveMotor.getEncoder();
    this.steerEncoder = this.steerMotor.getAbsoluteEncoder(); // REV Through Bore Encoder

    this.chassisAngularOffset = chassisAngularOffset;

    configure();

    resetEncoders();
  }

  public void configure() {

    this.driveMotor.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.steerMotor.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // 這個方法對應範本的 getAbsoluteEncoderPosition
  // 但加入了 Offset 計算，這是 MAXSwerve 必要的
  public double getAbsoluteEncoderRadians() {
    // 直接從 Spark Max 讀取絕對角度 (Radians)
    // 注意：Configs 裡已經設定了 PositionConversionFactor 為 2*PI，所以讀出來直接是弧度
    return this.steerEncoder.getPosition();
  }

  public void resetEncoders() {
    this.driveEncoder.setPosition(0);
    // MAXSwerve 的轉向是用絕對編碼器閉迴路，不需要像 CANcoder 那樣重設相對編碼器
    // 但我們可以更新初始狀態
    this.desiredState.angle = new Rotation2d(getSteerPosition());
  }

  public double getDrivePosition() {
    return this.driveEncoder.getPosition();
  }

  // 取得轉向角度 (已扣除 Offset)
  public double getSteerPosition() {
    return this.steerEncoder.getPosition() - this.chassisAngularOffset;
  }

  public double getDriveVelocity() {
    return this.driveEncoder.getVelocity();
  }

  public double getSteerVelocity() {
    return this.steerEncoder.getVelocity();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        this.getDrivePosition(),
        new Rotation2d(this.getSteerPosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        this.getDriveVelocity(),
        new Rotation2d(this.getSteerPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    // 1. 修正 Offset：將目標角度加上 Offset，還原成馬達該轉到的真實角度
    SwerveModuleState correctedState = new SwerveModuleState();
    correctedState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedState.angle = state.angle.plus(Rotation2d.fromRadians(this.chassisAngularOffset));

    // 2. 最佳化路徑 (Optimize)
    // 注意：這裡要傳入的是「尚未扣除 Offset」的真實編碼器角度，這樣比較才準確
    correctedState.optimize(new Rotation2d(this.steerEncoder.getPosition()));

    // 3. 設定馬達輸出
    this.driveController.setSetpoint(correctedState.speedMetersPerSecond, ControlType.kVelocity);
    this.steerController.setSetpoint(correctedState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = state;
  }

  public void stop() {
    this.driveMotor.set(0);
    this.steerMotor.set(0);
  }
  
}