// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        // public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxSpeedMetersPerSecond = 6.5;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(23.43);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.43);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final Translation2d[] autoLocations = new Translation2d[] {
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Index 3: BR (-X, -Y) 右後
        };

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kRearLeftDrivingCanId = 4;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 3;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 44;
        public static final int kFrontRightTurningCanId = 22;
        public static final int kRearRightTurningCanId = 33;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 16;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = 3.56;//(45.0 * 19) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 6782;
        // public static final double kFreeSpeedRpm = 5676;
    }

    public static final class FieldConstants {
        public static final Pose2d SHOOTING_POSTION = new Pose2d(2.5, 4.0, Rotation2d.fromDegrees(0));
        private static final AprilTagFieldLayout layout;
        public static final double fieldLength;
        public static final double fieldWidth;
        static {
            try {
                // 載入預設場地 (例如 2025 Reefscape 或 2026)
                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            } catch (Exception e) {
                throw new RuntimeException("地圖載入失敗", e);
            }
        }
        static {
            AprilTagFieldLayout layout;
            try {
                // 自動載入當年度的預設場地 (例如 2026 場地)
                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            } catch (Exception e) {
                // 萬一讀不到檔案 (極少發生)，給個預設值防止程式崩潰
                // 這裡可以填入規則書上的大約數值
                layout = null;
                e.printStackTrace();
            }

            if (layout != null) {
                // 從官方資料直接抓取精確數值
                fieldLength = layout.getFieldLength();
                fieldWidth = layout.getFieldWidth();
            } else {
                // Fallback (保底數值)
                fieldLength = 16.54;
                fieldWidth = 8.21;
            }

        }

        public class siteConstants {
            // Dimensions
            public static final double width = Units.inchesToMeters(31.8);
            public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
            public static final double height = Units.inchesToMeters(7.0);
            public static final double bumpers = Units.inchesToMeters(73.0);
            public static final double hub = Units.inchesToMeters(47.0);
            public static final double fieldWidth = 8.21; // 確保這裡能抓到數值

            /**
             * 安全地獲取目標點位。
             * 
             * @param type 0: Center, 1: Left, 2: Right
             */
            public static Translation3d getTargetPoint(int type) {
                double xPose;

                // 1. 安全檢查：如果 layout 沒載入或找不到 Tag 26，給一個預設的 X 座標
                if (layout != null && layout.getTagPose(26).isPresent()) {
                    xPose = layout.getTagPose(26).get().getX() + width / 2.0;
                } else {
                    // 如果找不到 Tag，先回傳一個靠近場地邊緣的預設 X (例如 1.0)
                    // 這樣至少 AutoTurn 會有一個目標，而不是直接讓程式當機
                    xPose = 1.0;
                }

                double yPose = fieldWidth / 2.0;
                double yOffset = (bumpers / 2 + hub / 2);

                switch (type) {
                    case 1: // Left
                        return new Translation3d(xPose, yPose + yOffset, height);
                    case 2: // Right
                        return new Translation3d(xPose, yPose - yOffset, height);
                    case 0: // Center
                    default:
                        return new Translation3d(xPose, yPose, height);
                }
            }
        }

        // 使用安全的方式取得 Tag 26 的 X 座標
        /*
         * private static double getTag26X() {
         * if (layout != null && layout.getTagPose(26).isPresent()) {
         * return layout.getTagPose(26).get().getX();
         * }
         * return 0.0; // 如果找不到 Tag 26，回傳 0，避免崩潰
         * }
         * 
         * public static final Translation3d topCenterPoint = new Translation3d(
         * getTag26X() + width / 2.0,
         * 4.105, // 直接給場地寬度的一半 (8.21 / 2)
         * 0.5 // 高度
         * );
         * 
         * public static final Translation3d topCenterPoint = new Translation3d(
         * layout.getTagPose(26).get().getX() + width / 2.0,
         * fieldWidth / 2.0, // Y 軸置中
         * height // 高度固定
         * );
         * public static final Translation3d topLeftCenterPoint = new Translation3d(
         * layout.getTagPose(26).get().getX() + width / 2.0,
         * (fieldWidth / 2.0) + (bumpers / 2 + hub / 2), // Y 軸置中
         * height // 高度固定
         * );
         * public static final Translation3d topRightCenterPoint = new Translation3d(
         * layout.getTagPose(26).get().getX() + width / 2.0,
         * (fieldWidth / 2.0) - (bumpers / 2 + hub / 2), // Y 軸置中
         * height // 高度固定
         * );
         */

    }

    public static final class LimelightConstants {
        public static final double MAX_GYRO_RATE = 1080;
    }

    public static final class ControllerConstants {
        public static final double INTAKE_SPEED = -
        0.35;
        public static final double INTAKE2_SPEED = -0.8;
        public static final double OUTTAKE_SPEED = -0.22;
        public static final double ELEVATOR_UP_LIMIT = -500;
        public static final double ELEVATOR_DOWN_LIMIT = 100;
        public static final double ARM_UP_LIMIT = 5;
        public static final double ARM_DOWN_LIMIT = 88;
        public static final double ARM_DOWN = 72;
        public static final double L1 = -280;
        public static final double LOWEST = -50;
        public static final double CLIMBER_UP = -24;
        public static final double CLIMBER_DOWN = 50;
        public static final double FEEDER_SPEED = 0.80;
        public static final double OUTFEED_SPEED = 0.1;
        public static final double FLYWHEEL_RPS = 40.0;// 49
        public static final double SHOOTER_SPEED = 1.0;
        public static final double SHOOT_AUTO_SPEED = 1.0;

    }

}
