package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class ClawSubsystem {

    private Servo  armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;

    private DcMotor rightSlidesMotor, leftSlidesMotor, intakeMotor;

    /** This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method. */

    public ClawSubsystem(HardwareMap hardwareMap) {
        armRotate = hardwareMap.get(Servo.class, "armServo");
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        linearSlides = hardwareMap.get(Servo.class, "linearSlides");
        grabby = hardwareMap.get(Servo.class, "grabby");
        rightSlideArm = hardwareMap.get(Servo.class, "rightSlideArm");
        leftSlideArm = hardwareMap.get(Servo.class, "leftSlideArm");
        rightSlidesMotor = hardwareMap.get(DcMotor.class, "rightSlidesMotor");
        leftSlidesMotor = hardwareMap.get(DcMotor.class, "leftSlidesMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

    }


    public double getLeftIntake(){return leftIntake.getPosition();}

    public double getRightIntake(){return rightIntake.getPosition();}

    public double getLinearSlides(){return linearSlides.getPosition();}

    public double getRightLideArm(){return rightSlideArm.getPosition();}

    public double getLeftSlideArm(){return leftSlideArm.getPosition();}

    public double getArmRotate(){return armRotate.getPosition();}

    public double getGrabby(){return grabby.getPosition();}

    public double getRightSlidesMotor(){return rightSlidesMotor.getCurrentPosition();}

    public double getLeftSlidesMotor(){return leftSlidesMotor.getCurrentPosition();}

    public double getIntakeMotor(){return intakeMotor.getCurrentPosition();}

    //------------------------------New Claw------------------------------//

    /**

    public void groundClaw() {pivot.setPosition(RobotConstants.groundClaw);}

     **/

    public void intakeDown() {
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightIntake.setPosition(RobotConstants.intakeDown);
        leftIntake.setPosition(RobotConstants.intakeDown);
    }

    public void  intakeUp(){
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightIntake.setPosition(RobotConstants.intakeUp);
        leftIntake.setPosition(RobotConstants.intakeUp);
    }

    public void linearSlidesIn() {linearSlides.setPosition(RobotConstants.linearSlidesIn);}

    public void slidesArmInIntake() {
        rightSlideArm.setPosition(RobotConstants.slidesArmInIntake);
        leftSlideArm.setPosition(RobotConstants.slidesArmInIntake);
    }

    public void grabbyOpen() {grabby.setPosition(RobotConstants.grabbyOpen);}

    public void grabbyClose() {grabby.setPosition(RobotConstants.grabbyClose);}

    public void slidesDown() {
        rightSlidesMotor.setTargetPosition(RobotConstants.slidesDown);
        leftSlidesMotor.setTargetPosition(RobotConstants.slidesDown);
    }
    public void readyClip() {
        leftSlideArm.setPosition(.25);
        rightSlideArm.setPosition(.25);
        armRotate.setPosition(.54);
        rightSlidesMotor.setTargetPosition(-5);
        leftSlidesMotor.setTargetPosition(-20);
    }
    public void clip(){
        rightSlidesMotor.setPower(.8);
        leftSlidesMotor.setPower(.8);
        rightSlidesMotor.setTargetPosition(-700);
        leftSlidesMotor.setTargetPosition(-700);
        leftSlideArm.setPosition(.1);
        rightSlideArm.setPosition(.1);
        armRotate.setPosition(.90);
    }
    public void getClip(){
        leftSlideArm.setPosition(.78);
        rightSlideArm.setPosition(.78);
        armRotate.setPosition(.60);
        rightSlidesMotor.setTargetPosition(-5);
        leftSlidesMotor.setTargetPosition(-20);
    }

    public void intakeIn() {intakeMotor.setPower(RobotConstants.intakeIn);}

    public void intakeStop() {intakeMotor.setPower(RobotConstants.intakeStop);}

    public void intakeOut() {intakeMotor.setPower(RobotConstants.intakeOut);}

    //public void armGetClip() {armRotate.setPosition();}

}