package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.DriveAndAlignCommand;
import frc.robot.commands.FeederCmd;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.autoRotation;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.joystick.Controller;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.commands.FlywheelCmd;
import frc.robot.subsystems.Flywheel;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Driver driver = new Driver(0);
        private final SendableChooser<Command> autoChooser;
        private final Field2d field = new Field2d();
        private final LimeLight limelight = new LimeLight(swerveSubsystem, "limelight");
        private final Intake intake = new Intake();
        private final Elevator elevator = new Elevator();
        private final Arm arm = new Arm();
        private final Feeder feeder = new Feeder();
        private final Shooter shooter = new Shooter();
        private final Controller controller = new Controller();
        private final Flywheel flywheel = new Flywheel();

        public RobotContainer() {
                configureBindings();
                this.registerCommands();

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                driver));

                this.intake.setDefaultCommand(new IntakeCmd(
                                this.intake,
                                this.controller::isIntake,
                                this.controller::isOuttake));

                this.elevator.setDefaultCommand(new ElevatorCmd(
                                this.elevator,
                                this.driver::isUp,
                                this.driver::isDown,
                                this.driver::L1,
                                this.driver::Lowest
                ));
                this.arm.setDefaultCommand(new ArmCmd(
                                this.arm,
                                this.controller::getPOVAngle));

                this.feeder.setDefaultCommand(new FeederCmd(
                                this.feeder,
                                this.controller::isFeed,
                                this.controller::isOutFeed));
                this.shooter.setDefaultCommand(new ShooterCmd(
                                this.shooter,
                                this.controller::isShoot // 假設你的 Controller 類別有這些方法
                ));
                this.flywheel.setDefaultCommand(new FlywheelCmd(
                                this.flywheel,
                                this.controller::isFly)); // 這裡你可以根據需要調整目標 RPS

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
        }

        public void log() {
                SmartDashboard.putData("Field", field);

                // Logging callback for current robot pose
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.setRobotPose(pose);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.getObject("target pose").setPose(pose);
                });

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        field.getObject("path").setPoses(poses);
                });

        }

        private void registerCommands() {
                NamedCommands.registerCommand("armMoveToDown", this.arm.armMoveToDown());
                NamedCommands.registerCommand("shoot", this.shooter.shoot());
                NamedCommands.registerCommand("flywheelShoot", this.flywheel.flyWheelShoot(85));
                NamedCommands.registerCommand("moveToL1", this.elevator.moveToL1());
                NamedCommands.registerCommand("feed", this.feeder.feed());
                NamedCommands.registerCommand("MoveToLowest", this.elevator.moveToLowest());
                
        }

        private void configureBindings() {
                // 按下 START 鍵歸零陀螺儀
                new JoystickButton(driver, XboxController.Button.kStart.value)
                                .onTrue(swerveSubsystem.runOnce(() -> swerveSubsystem.zeroHeading()));
                // --- 新增：按住 A 鍵進行視覺平移對準 ---
                new JoystickButton(driver, XboxController.Button.kB.value)
                                .whileTrue(new autoRotation(swerveSubsystem, limelight));
                // 這樣 9079 的隊員只要按住 A，機器人就會自動吸附到 HUB 前面的射球位
                new JoystickButton(driver, XboxController.Button.kA.value)
                                .whileTrue(new DriveAndAlignCommand(swerveSubsystem, limelight));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public LimeLight geLimeLight() {
                return limelight;
        }

}