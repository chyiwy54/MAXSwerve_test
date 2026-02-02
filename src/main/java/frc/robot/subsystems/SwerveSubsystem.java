// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.autoLocations);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      gyro.getRotation2d(), // Phoenix 6 的 Pigeon2 直接支援 getRotation2d()
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      });

  public SwerveSubsystem() {
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // 告訴 AutoBuilder 機器人在哪 (Pose2d)
        this::resetOdometry, // 告訴 AutoBuilder 如何重設位置
        this::getRobotRelativeSpeeds, // ❌ 你原本沒有這個方法，請看下面第 3 點新增
        (speeds, feedforwards) -> autorunVelocity(speeds), // ✅ 使用你寫好的 robot-relative 驅動方法
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID (請依照實際情況調整)
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (請依照實際情況調整)
        ),
        config,
        () -> {
          // 自動判斷是否為紅隊 (路徑翻轉)
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Subsystem 參考
    );
    
    Pose2d initialPose = new Pose2d(0, 0, this.getRotation2d());
    this.poseEstimator = new SwerveDrivePoseEstimator(
        this.kinematics,
        this.getRotation2d(),
        this.getModulePositions(),
        initialPose);
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
    // rad = -rad; // 視情況取負號
    return new Rotation2d(rad);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void autorunVelocity(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
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
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.05);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.05);
    rot = MathUtil.applyDeadband(rot, 0.05);

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

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

    // 注意：如果你已經換成新的 SwerveModule 類別，這裡要用 .setState()
    // 如果還是舊的 MAXSwerveModule，則維持 .setDesiredState()
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
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
}
