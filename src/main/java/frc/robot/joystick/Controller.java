package frc.robot.joystick;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends XboxController {
    public Controller() {
        super(1);
    }

    public double isIntake() {
        return this.getLeftTriggerAxis();
    }

    public boolean isOuttake() {
        return this.getRawButton(5);
    }

    public boolean isFeed() {
        return this.getRawButton(2);
    }

    public boolean isOutFeed() {
        return this.getRawButton(3);
    }
    
    public double isShoot() {
        return this.getRightTriggerAxis();
    }
    
    public int getPOVAngle() {
        return this.getPOV();
    }

    public boolean isFly() {
        return this.getRawButton(6);
    }

    public boolean isRise() {
        return this.getRawButton(7);
    }


  

}