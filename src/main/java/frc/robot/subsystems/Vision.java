package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private static final NetworkTableEntry rightTx = NetworkTableInstance.getDefault()
        .getTable("limelight-downr").getEntry("tx");
    private static final NetworkTableEntry rightTy = NetworkTableInstance.getDefault()
        .getTable("limelight-downr").getEntry("ty");
    private static final NetworkTableEntry rightTa = NetworkTableInstance.getDefault()
        .getTable("limelight-downr").getEntry("ta");
    private static final NetworkTableEntry rightId = NetworkTableInstance.getDefault()
        .getTable("limelight-downr").getEntry("tid");

    private static final NetworkTableEntry leftTx = NetworkTableInstance.getDefault()
        .getTable("limelight-downl").getEntry("tx");
    private static final NetworkTableEntry leftTy = NetworkTableInstance.getDefault()
        .getTable("limelight-downl").getEntry("ty");
    private static final NetworkTableEntry leftTa = NetworkTableInstance.getDefault()
        .getTable("limelight-downl").getEntry("ta");
    private static final NetworkTableEntry leftId = NetworkTableInstance.getDefault()
        .getTable("limelight-downl").getEntry("tid");

    public static double getRightTx() { return rightTx.getDouble(0); }
    public static double getRightTy() { return rightTy.getDouble(0); }
    public static double getRightTa() { return rightTa.getDouble(0); }
    public static double getRightId() { return rightId.getDouble(0); }

    public static double getLeftTx() { return leftTx.getDouble(0); }
    public static double getLeftTy() { return leftTy.getDouble(0); }
    public static double getLeftTa() { return leftTa.getDouble(0); }
    public static double getLeftId() { return leftId.getDouble(0); }

    // 確認是否有偵測到目標
    public static boolean hasTargetRight() { return getRightTa() > 0; }
    public static boolean hasTargetLeft() { return getLeftTa() > 0; }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Right Limelight Has Target", hasTargetRight());
        SmartDashboard.putBoolean("Left Limelight Has Target", hasTargetLeft());

        SmartDashboard.putNumber("Right Limelight X", getRightTx());
        SmartDashboard.putNumber("Right Limelight Y", getRightTy());
        SmartDashboard.putNumber("Right Limelight Area", getRightTa());
        SmartDashboard.putNumber("Right AprilTag ID", getRightId());

        SmartDashboard.putNumber("Left Limelight X", getLeftTx());
        SmartDashboard.putNumber("Left Limelight Y", getLeftTy());
        SmartDashboard.putNumber("Left Limelight Area", getLeftTa());
        SmartDashboard.putNumber("Left AprilTag ID", getLeftId());

    }
}