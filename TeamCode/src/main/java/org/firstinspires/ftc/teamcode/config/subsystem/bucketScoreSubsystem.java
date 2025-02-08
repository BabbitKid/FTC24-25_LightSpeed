package org.firstinspires.ftc.teamcode.config.subsystem;

import static java.lang.Thread.onSpinWait;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class bucketScoreSubsystem {

    public boolean slidesToBucket;
    private Servo  armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;

    private DcMotor rightSlidesMotor, leftSlidesMotor, intakeMotor;



    /** This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method. */

    public bucketScoreSubsystem(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        rightSlidesMotor = hardwareMap.dcMotor.get("rightSlidesMotor");
        leftSlidesMotor = hardwareMap.dcMotor.get("leftSlidesMotor");

        grabby = hardwareMap.servo.get("grabby");


        linearSlides = hardwareMap.servo.get("linearSlides");
        armRotate = hardwareMap.servo.get("armServo");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        rightSlideArm = hardwareMap.servo.get("rightSlideArm");
        leftSlideArm = hardwareMap.servo.get("leftSlideArm");


        rightSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightSlideArm.setDirection(Servo.Direction.REVERSE);



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


    public void grabbyClose() {grabby.setPosition(RobotConstants.grabbyClose);}

    public void slidesDown() {
        rightSlidesMotor.setTargetPosition(RobotConstants.slidesDown);
        leftSlidesMotor.setTargetPosition(RobotConstants.slidesDown);
    }

    public boolean slidesToBucket() throws InterruptedException {
        rightSlidesMotor.setTargetPosition(-2800);
        leftSlidesMotor.setTargetPosition(-2800);
        leftSlideArm.setPosition(.32);
        rightSlideArm.setPosition(.32);
        rightSlidesMotor.setPower(.8);
        leftSlidesMotor.setPower(.8);

        Thread.sleep(2300);
        armRotate.setPosition(.39);
        Thread.sleep(800);
        grabby.setPosition(.3); //Let block go
        Thread.sleep(800);
        leftSlideArm.setPosition(.87);
        rightSlideArm.setPosition(.87);




        if (rightSlidesMotor.getCurrentPosition() > -55 && rightSlidesMotor.getCurrentPosition() < -55) {
            return true;
        }
        return false;


    };
   public void slidesDown1() {
       rightSlidesMotor.setTargetPosition(-20);
       leftSlidesMotor.setTargetPosition(-20);
       rightSlidesMotor.setPower(.45);
       leftSlidesMotor.setPower(.45);
       leftSlideArm.setPosition(.87);
       armRotate.setPosition(.61);
   }
    public void intakeRotate() throws InterruptedException{
        rightSlideArm.setPosition(.9);
        leftSlideArm.setPosition(.9);
        armRotate.setPosition(.61);
        intakeMotor.setPower(-1);


    }
    public void grabberandintake(){
        grabby.setPosition(.6);
        intakeMotor.setPower(0);
    }

    // add slides up

    public void intakeIn() {intakeMotor.setPower(RobotConstants.intakeIn);}

    public void intakeStop() {intakeMotor.setPower(RobotConstants.intakeStop);}

    public void intakeOut() {intakeMotor.setPower(RobotConstants.intakeOut);}

    //public void armGetClip() {armRotate.setPosition();}

}