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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp(name="IntoTheDeep Two Driver TeleOp", group="Linear Opmode")
public class IntoTheDeepTwoDriverTeleOp extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private static DcMotor leftFront, leftBack, rightFront, rightBack, slideMotor;

    private Servo intake, intakeArm, intakeSlides,fourBar, dropper;
    private static double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
    private static double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    private double ENCODER_TICKS_PER_ROTATION = 1120 * (2.0/3);
    private double driveFactor = 1;
    private boolean isGoingAllTheWayUp = false;




    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        intake = hardwareMap.servo.get("intake");
        intakeArm = hardwareMap.servo.get("intakeArm");
        intakeSlides = hardwareMap.servo.get("intakeSlides");
        fourBar = hardwareMap.servo.get("fourBar");
        dropper = hardwareMap.servo.get("dropper");
        slideMotor = hardwareMap.dcMotor.get("vertSlides");


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(90));

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(.75);

            leftJoystickX = gamepad1.left_stick_x;
            leftJoystickY = gamepad1.left_stick_y;
            rightJoystickX = gamepad1.right_stick_x;
            rightJoystickY = gamepad1.right_stick_y;

            leftFrontPower =    leftJoystickY - leftJoystickX - rightJoystickX;
            rightFrontPower = -leftJoystickY - leftJoystickX - rightJoystickX;
            leftBackPower = leftJoystickY + leftJoystickX - rightJoystickX;
            rightBackPower = -leftJoystickY + leftJoystickX - rightJoystickX;

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
            if (gamepad1.right_trigger > .5) {
                fourBar.setPosition(.45); //four bar out
            }
            if (gamepad1.left_trigger > .5) {
                fourBar.setPosition(0); //four bar in
            }
            if (gamepad1.a) { //dropper with wiggle
                dropper.setPosition(.33);
                dropper.setPosition(.35);
                dropper.setPosition(.37);
            } else dropper.setPosition(.5);



            if (gamepad2.left_trigger > .5) {
                intakeArm.setPosition(.64); //full down
            }
            if (gamepad2.left_bumper) {
                intakeArm.setPosition(.51); //hover arm
            }

            if (gamepad2.b) {
                intakeArm.setPosition(0); //full in robot
            }
            if (gamepad2.right_trigger > .1) {
                intake.setPosition(0); //grab
            } else {
                intake.setPosition(1); //release
            }
            if (gamepad2.dpad_down) {
                intakeSlides.setPosition(.28); //slides in robot
            }
            if (gamepad2.dpad_up) {
                intakeSlides.setPosition(.69); //slides extended
            }
            if (gamepad2.x) {

            }


            // zero out the slides position
            /**if (gamepad1.b) {
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/



            if (gamepad2.y) {
                slideMotor.setPower(.75);
                slideMotor.setTargetPosition(5500);
            }
            if (gamepad2.a) {
                slideMotor.setPower(.5);
                slideMotor.setTargetPosition(convertDegreesToEncoderTicks(85)); //slide all the way down
            }
            if (gamepad2.dpad_left) {
                slideMotor.setPower(.5);
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition()+convertDegreesToEncoderTicks(15));
            }


            /**if (gamepad2.dpad_right){
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition()+convertDegreesToEncoderTicks(20));
            }
            if (gamepad2.dpad_left){
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition()-convertDegreesToEncoderTicks(20));
            }
            if (gamepad1.dpad_up){
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition()+convertDegreesToEncoderTicks(20));
            }
            if (gamepad1.dpad_down){
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition()-convertDegreesToEncoderTicks(20));
            }*/

            telemetry.addData("currentMotorPosition", slideMotor.getCurrentPosition());
            telemetry.update();








        }

    }

    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }

}