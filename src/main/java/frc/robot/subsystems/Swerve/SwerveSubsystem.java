// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import static edu.wpi.first.units.Units.Radians;

public class SwerveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule backLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule backRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(6);

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d m_field = new Field2d();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.autoLocations);
  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> realSwerve = NetworkTableInstance.getDefault()
      .getStructTopic("realSwervepose", Pose2d.struct).publish();
  StructPublisher<Pose2d> simPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("simSwerveposed", Pose2d.struct).publish();

  private Pose2d simOdometry = new Pose2d();
  private double posedX = 0;
  private double posedY = 0;
  private double posedRot = 0;
  private double timeFromLastUpdate = 0;
  private double lastSimTime = 0;

  public SwerveSubsystem() {
    // 1. 新增：陀螺儀開機延遲保護 (等待 1 秒確保 Pigeon2 暖機完成後再歸零)
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    // 2. 順序調換：必須先初始化 poseEstimator！
    // 這樣後面的 AutoBuilder 呼叫 getPose 時才不會抓到空值而當機
    Pose2d initialPose = new Pose2d(0, 0, this.getRotation2d());
    this.poseEstimator = new SwerveDrivePoseEstimator(
        this.kinematics,
        this.getRotation2d(),
        this.getModulePositions(),
        initialPose);

    // 3. 讀取 PathPlanner 的 GUI 設定
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // 4. 設定新版 AutoBuilder
    AutoBuilder.configure(
        this::getPose, // 現在這裡安全了，抓得到剛剛建立的 poseEstimator！
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> autorunVelocity(speeds), // 相容新版寫法
        new PPHolonomicDriveController(
            new PIDConstants(5.00, 0.0, 0.105),//0.105
            new PIDConstants(0.5, 0.0, 0.0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // 5. 完美保留你寫的 SmartDashboard 數據區塊
    SmartDashboard.putData("SwerveDrive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("FrontLeft Posisiton", () -> frontLeft.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("FrontLeft Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("FrontRight Posisiton", () -> frontRight.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("FrontRight Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("BackLeft Posisiton", () -> backLeft.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("BackLeft Velocity", () -> backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("BackRight Posisiton", () -> backRight.getPosition().angle.getRadians(), null);
        builder.addDoubleProperty("BackRight Velocity", () -> backRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Heading", () -> getRotation2d().getRadians(), null);
      }
    });
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // 將 4 顆輪子的狀態轉回底盤的 X, Y, Omega 速度
    return kinematics.toChassisSpeeds(this.getModuleStates());
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public double getHeading() {
    StatusSignal<Angle> yawSignal = gyro.getYaw();
    double yawValue = yawSignal.getValueAsDouble();
    return Math.IEEEremainder(yawValue, 360);
  }

  public Rotation2d getRotation2d() {
    var yaw = gyro.getYaw();
    yaw.refresh();
    double rad = yaw.getValue().in(Radians);

    // 關鍵修正：
    // 如果是紅色聯盟，我們需要把角度加上 180 度，讓機器人的前方對準場地的 X 軸正方向
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return new Rotation2d(rad).plus(Rotation2d.fromDegrees(180));
    }

    return new Rotation2d(rad);
  }

  public Pose2d getPose() {
    // 🌟 關鍵：模擬時讀取虛擬座標
    if (Robot.isSimulation()) {
      return this.simOdometry;
    }
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        getRotation2d(),
        getModulePositions());
    getPose();

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(getPose());

    publisher.set(getModuleStates());
    simPublisher.set(simOdometry);
    realSwerve.set(poseEstimator.getEstimatedPosition());

  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void autorunVelocity(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    // --- 新增：自動階段的模擬計算 ---
    if (Robot.isSimulation()) {
      // 💡 注意：PathPlanner 傳來的是「機器人相對速度」，但場地模擬需要「場地相對速度」
      // 所以我們先把它轉換回場地視角，再丟給 simDrive 計算！
      ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getRotation2d());
      simDrive(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond, fieldSpeeds.omegaRadiansPerSecond);
    }
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    this.setModuleStates(moduleStates);

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        this.frontLeft.getState(),
        this.frontRight.getState(),
        this.backLeft.getState(),
        this.backRight.getState()
    };
  }

  public void resetOdometry(Pose2d pose) {
    // 🌟 關鍵：重設位置時，虛擬變數和真實里程計都要一起更新
    if (Robot.isSimulation()) {
      this.simOdometry = pose;
      this.posedX = pose.getX();
      this.posedY = pose.getY();
      this.posedRot = pose.getRotation().getRadians();
    }
    poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
    rot = MathUtil.applyDeadband(rot, 0.05);

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    // --- 新增：搖桿開車時的模擬計算 ---
    if (Robot.isSimulation()) {
      // 搖桿傳進來的通常是場地相對速度 (Field-Relative)
      simDrive(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    // 2. 移除三元運算子 (? :)，直接使用 fromFieldRelativeSpeeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered,
            ySpeedDelivered,
            rotDelivered,
            gyro.getRotation2d() // 必須確保 gyro 正常運作，否則方向會亂
        ));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    this.setModuleStates(swerveModuleStates);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    this.poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
  }
  public Pigeon2 getPigeon2() {
    return this.gyro;
  }

  // --- 新增：模擬環境專用的運動學計算 ---
  public void simDrive(double fieldXSpeed, double fieldYSpeed, double rotSpeed) {
    double currentTime = Timer.getFPGATimestamp();
    // 避免第一次啟動時時間差過大
    if (lastSimTime == 0)
      lastSimTime = currentTime;

    double dt = currentTime - lastSimTime;
    lastSimTime = currentTime;

    // 積分計算：速度 * 時間差 = 位移
    this.posedX += fieldXSpeed * dt;
    this.posedY += fieldYSpeed * dt;
    this.posedRot += rotSpeed * dt;

    this.simOdometry = new Pose2d(this.posedX, this.posedY, new Rotation2d(this.posedRot));
  }

}
