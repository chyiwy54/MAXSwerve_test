package frc.robot.joystick;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Shooter;

public class Controller extends XboxController {
    public Controller() {
        super(1);
    }

    public boolean isIntake() {
        return this.getRawButton(5);
    }
     public boolean isIntake2() {
        return this.getRawButton(1);
    }

    public double isOuttake() {
        return this.getLeftTriggerAxis();
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


   /*  public boolean isUp() {
        return this.getRawButton(1);
    }*/

   /*  public boolean isDown() {
        return this.getRawButton(2);
    }*/

    /*public boolean L2() {
        return this.getRawButton(8);
    }

    public boolean L3() {
        return this.getRawButton(10);
    }

    public boolean Lowest() {
        return this.getRawButton(11);
    }*/

    /*public boolean armLowest() {
        return this.getRawButton(11);
    }*/

   /*  public double getTurnValue() {
        return this.getRawAxis(3);
    }*/

    public boolean isRise() {
        return this.getRawButton(7);
    }


  

}