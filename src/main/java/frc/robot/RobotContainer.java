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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.joystick.Driver;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
       private final SwerveSubsystem swerve = new SwerveSubsystem();
        private final Driver  driver = new Driver();
        private final Vision vision = new Vision();
        private final SendableChooser<Command> autoChooser;

 
        public RobotContainer() {
                configureButtonBindings();
                this.swerve.setDefaultCommand(new SwerveJoystickCmd(
                                this.swerve,
                                this.driver::getXLimiter,
                                this.driver::getYLimiter,
                                this.driver::getTurningLimiter,
                                this.driver::getLeftBumperButton));
               
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Mode", autoChooser);
        }

        
        private void configureButtonBindings() {
                new Trigger(this.driver::getStartButton)
                                .onTrue(new InstantCommand(() -> swerve.zeroHeading()));
                new Trigger(this.driver::getXButton)
                                .onTrue(new InstantCommand(() -> swerve.setX()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}
