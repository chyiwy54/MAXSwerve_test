// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private final PIDController pidController = new PIDController(0.2, 0.0, 0.0);
  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> realSwerve = NetworkTableInstance.getDefault()
      .getStructTopic("realSwervepose", Pose2d.struct).publish();
  StructPublisher<Pose2d> simPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("simS werveposed", Pose2d.struct).publish();
  private final Field2d m_field = new Field2d();

  private Pose2d simOdometry;
  private double posedX = 0;
  private double posedY = 0;
  private double posedRot = 0;
  private double timeFromLastUpdate = 0;
  private double lastSimTime = 0;

  /** Creates a new DriveSubsystem. */
  public SwerveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) {
      }
    }).start();

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
    this.simOdometry = this.odometry.getPoseMeters();
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new PPHolonomicDriveController(
            new PIDConstants(0.5, 0.0, 0.0),
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

  public ChassisSpeeds getChassisSpeeds() {

    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    if (Robot.isSimulation()) {
      this.simDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      this.setModuleStates(states);
      // SmartDashboard.putString("badddddd", speeds.toString());
      // SmartDashboard.putNumber("test", speeds.vxMetersPerSecond);
    } else {
      SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      setModuleStates(states);
    }
  }

  public void zeroHeading() {
    gyro.reset();
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
    if (Robot.isSimulation())
      return this.simOdometry;
    else
      return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    this.updateTime();
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    SmartDashboard.putNumber("Robot Heading", getHeading());
    // SmartDashboard.putString("Robot Location",
    // getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Location", getPose().toString());
    this.realSwerve.set(this.getPose());
    this.simPublisher.set(this.simOdometry);
    this.publisher.set(this.getModuleStates());

    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(getPose());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void driveSwerve(double xSpeed, double ySpeed, double rotSpeed) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getRotation2d());
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    this.setModuleStates(moduleStates);
  }

  public void autoTurning(double xSpeed, double ySpeed, double angle) {
    double speed = (angle == 0) ? 0 : this.pidController.calculate(angle, 0);
    this.driveSwerve(xSpeed, ySpeed, speed);
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
    // 有手寫 simOdometry 變數，也要同步
    this.simOdometry = pose;
    this.posedX = pose.getX();
    this.posedY = pose.getY();
    this.posedRot = pose.getRotation().getRadians();
  }

  public void simDrive(double xspeed, double ySpeed, double rotSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(xspeed, ySpeed, rotSpeed);
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    this.setModuleStates(states);
    // SmartDashboard.putNumber("test/x", speeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("test/y", speeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("test/rot", speeds.omegaRadiansPerSecond);
    this.publisher.set(states);
    if (speeds.vxMetersPerSecond != 0) {
      this.posedX += speeds.vxMetersPerSecond * timeFromLastUpdate;
    }
    if (speeds.vyMetersPerSecond != 0) {
      this.posedY += speeds.vyMetersPerSecond * timeFromLastUpdate;
    }
    if (speeds.omegaRadiansPerSecond != 0) {
      this.posedRot += speeds.omegaRadiansPerSecond * timeFromLastUpdate;
    }
    this.simOdometry = new Pose2d(this.posedX, this.posedY, new Rotation2d(this.posedRot));
  }

  /**
   * 使用搖桿資訊驅動機器人。
   *
   * @param xSpeed        機器人在 x 方向（前進）的速度。
   * @param ySpeed        機器人在 y 方向（側向）的速度。
   * @param rot           機器人的旋轉速度。
   * @param fieldRelative 所提供的 x 和 y 速度是否相對於場地。
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // 將命令速度轉換為底盤的正確單位
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                gyro.getRotation2d()) // 直接使用 Pigeon 2 的 getRotation2d()
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    // 修正打字錯誤，並確保輪速不會超過最大限制
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // 設定每個模組的目標狀態
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void updateTime() {
    this.timeFromLastUpdate = Timer.getFPGATimestamp() - this.lastSimTime;
    this.lastSimTime = Timer.getFPGATimestamp();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** 獲取旋轉速率 (Degrees per second) */
  public double getTurnRate() {
    // 獲取角速度，注意這裡獲取的是 DegreesPerSecond
    return gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
