/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp(name="IntoTheDeep Two Driver TeleOp", group="Linear Opmode")
public class IntoTheDeepTwoDriverTeleOp extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private static DcMotor leftFront, leftBack, rightFront, rightBack, intakeMotor, rightSlidesMotor, leftSlidesMotor;

    private Servo armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;

    private static double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
    private static double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    private double ENCODER_TICKS_PER_ROTATION = 1120 * (2.0 / 3);
    private double driveFactor = 1;
    private boolean isGoingAllTheWayUp = false;


    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
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
        rightSlidesMotor.setTargetPosition(convertDegreesToEncoderTicks(5));
        rightSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlidesMotor.setTargetPosition(convertDegreesToEncoderTicks(5));
        leftSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightSlideArm.setDirection(Servo.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //grabby.setDirection(Servo.Direction.REVERSE);
        //armRotate.setDirection(Servo.Direction.REVERSE);

        /**

         leftIntake.setPosition(.47);
         rightIntake.setPosition(.47);
         linearSlides.setPosition(.7);
         rightSlideArm.setPosition(.9);
         leftSlideArm.setPosition(.9);
         armRotate.setPosition(.65);
         grabby.setPosition(.27);
         rightSlidesMotor.setTargetPosition(-10);
         leftSlidesMotor.setTargetPosition(-10);
         **/

        waitForStart();

        // run until the end of the match (driver presses STOP

        while (opModeIsActive()) {

            //grabby.setPosition(.47);


            //slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);   //////////////////////////////////////////////////
            //slideMotor.setPower(.75);
            leftJoystickX = gamepad1.left_stick_x;
            leftJoystickY = gamepad1.left_stick_y;
            rightJoystickX = gamepad1.right_stick_x;
            rightJoystickY = gamepad1.right_stick_y;

            double leftJoystickX = -gamepad1.left_stick_x;  // Keep strafing direction normal
            double leftJoystickY = -gamepad1.left_stick_y;  // Keep forward/backward correct
            double rightJoystickX = gamepad1.right_stick_x; // Keep rotation correct

// Adjust motor calculations: Flip leftJoystickX for the left-side motors
            double leftFrontPower = leftJoystickY - leftJoystickX + rightJoystickX; // Flipped strafing
            double rightFrontPower = leftJoystickY + leftJoystickX - rightJoystickX;
            double leftBackPower = leftJoystickY + leftJoystickX + rightJoystickX;  // Flipped strafing
            double rightBackPower = leftJoystickY - leftJoystickX - rightJoystickX;


            double[] wheelPower = {Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightFrontPower), Math.abs(rightBackPower)};
            Arrays.sort(wheelPower);
            double largestInput = wheelPower[3];
            if (largestInput > 1) {
                leftFrontPower /= largestInput;
                leftBackPower /= largestInput;
                rightFrontPower /= largestInput;
                rightBackPower /= largestInput;
            }

            if (gamepad1.right_bumper) {
                leftFront.setPower(leftFrontPower / 2);
                leftBack.setPower(leftBackPower / 2);
                rightFront.setPower(rightFrontPower / 2);
                rightBack.setPower(rightBackPower / 2);
            } else if (gamepad1.left_bumper) {
                leftFront.setPower(leftFrontPower / 4);
                leftBack.setPower(leftBackPower / 4);
                rightFront.setPower(rightFrontPower / 4);
                rightBack.setPower(rightBackPower / 4);
            } else {
                leftFront.setPower(leftFrontPower);
                leftBack.setPower(leftBackPower);
                rightFront.setPower(rightFrontPower);
                rightBack.setPower(rightBackPower);

            }


            //////////////////////////////////////////////////////////////////////////////////////////////////


            if (gamepad2.x) { //go clip pos
                leftSlideArm.setPosition(.25);
                rightSlideArm.setPosition(.25);
                armRotate.setPosition(.54);
                rightSlidesMotor.setTargetPosition(-5);
                leftSlidesMotor.setTargetPosition(-20);
            }
            if (gamepad2.b) { //in intake pos
                rightSlideArm.setPosition(.87);
                leftSlideArm.setPosition(.87);
                armRotate.setPosition(.61);
                rightSlidesMotor.setTargetPosition(-5);
                leftSlidesMotor.setTargetPosition(-20);
                timer.reset();
                while (opModeIsActive() && timer.seconds() < 1.5) {
                    idle();
                }
                rightSlideArm.setPosition(.93);
                leftSlideArm.setPosition(.93);
                armRotate.setPosition(.61);
                rightSlidesMotor.setTargetPosition(-5);
                leftSlidesMotor.setTargetPosition(-20);
            }
            if (gamepad2.y) { //get clip pos
                leftSlideArm.setPosition(.78);
                rightSlideArm.setPosition(.78);
                armRotate.setPosition(.60);
                rightSlidesMotor.setTargetPosition(-5);
                leftSlidesMotor.setTargetPosition(-20);
            }
            if (gamepad2.a) {
                rightSlidesMotor.setPower(.8);
                leftSlidesMotor.setPower(.8);
                rightSlidesMotor.setTargetPosition(-700);
                leftSlidesMotor.setTargetPosition(-700);
                leftSlideArm.setPosition(.1);
                rightSlideArm.setPosition(.1);
                armRotate.setPosition(.90);
            }

            /**

             this version is with the get clip on the back and is not working

             if (gamepad2.x) { //get clip pos
             leftSlideArm.setPosition(.06);
             rightSlideArm.setPosition(.06);
             armRotate.setPosition(.47);
             }
             if (gamepad2.b) { //in intake pos
             rightSlideArm.setPosition(.9);
             leftSlideArm.setPosition(.9);
             armRotate.setPosition(.65);
             }
             if (gamepad2.y) { //get clip pos
             leftSlideArm.setPosition(.60);
             rightSlideArm.setPosition(.60);
             armRotate.setPosition(.70);
             }
             if (gamepad2.a)  {
             leftSlideArm.setPosition(.25);
             rightSlideArm.setPosition(.25);
             armRotate.setPosition(.73);
             }

             if (gamepad2.right_trigger > .5) {
             rightSlidesMotor.setPower(.8);
             leftSlidesMotor.setPower(.8);
             rightSlidesMotor.setTargetPosition(-1000);
             leftSlidesMotor.setTargetPosition(-1000);
             }

             if (gamepad2.left_trigger > .5) {
             rightSlidesMotor.setPower(.8);
             leftSlidesMotor.setPower(.8);
             rightSlidesMotor.setTargetPosition(-20);
             leftSlidesMotor.setTargetPosition(-20);
             }

             */

            if (gamepad2.right_bumper) {
                rightSlidesMotor.setPower(.8);
                leftSlidesMotor.setPower(.8);
                rightSlidesMotor.setTargetPosition(-3350);
                leftSlidesMotor.setTargetPosition(-3350);
                leftSlideArm.setPosition(.2);
                rightSlideArm.setPosition(.2);
                armRotate.setPosition(.45);


            }

            if (gamepad2.left_bumper) {
                rightSlidesMotor.setPower(.8);
                leftSlidesMotor.setPower(.8);
                rightSlidesMotor.setTargetPosition(-5);
                leftSlidesMotor.setTargetPosition(-20);
            }


                if (gamepad2.dpad_up) {
                    linearSlides.setPosition(.2);
                }
                if (gamepad2.dpad_down) {
                    linearSlides.setPosition(.75);
                }


                if (gamepad1.right_trigger > .5) {
                    intakeMotor.setPower(-1);
                } else if (gamepad1.left_trigger > .5) {
                    intakeMotor.setPower(1);
                } else {
                    intakeMotor.setPower(0);
                }

                if (gamepad1.y) {
                    grabby.setPosition(.27);
                }
                if (gamepad1.a) {
                    grabby.setPosition(.6);
                }
                if (gamepad1.dpad_up) {
                    leftIntake.setPosition(0);
                    rightIntake.setPosition(0);
                }
                if (gamepad1.dpad_down) {
                    leftIntake.setPosition(.47);
                    rightIntake.setPosition(.47);
                }


                //Intake .25 is up and 0 is down
                // linear slides in .75 linear slides out 0
                //slide arm clipping is .1 in intake is .9

                //////////////////////////////////////////////////////////////////////////////////////

                telemetry.addData("servo pos", leftIntake.getPosition());
                telemetry.addData("servo pos", rightIntake.getPosition());
                telemetry.addData("slidesPosRight", rightSlidesMotor.getCurrentPosition());
                telemetry.addData("slidesPosLeft", leftSlidesMotor.getCurrentPosition());
                telemetry.update();


            }

        }

        public int convertDegreesToEncoderTicks ( double degrees){
            return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
        }
    }