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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="CenterStageAuto", group="Linear Opmode")
public class CenterStageAuto extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private static DcMotor leftFront, leftBack, rightFront, rightBack, slideMotor, intakeMotor;

    private Servo dropperServo, planeServo, rotateServo;

    private static double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
    private static double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    private double ENCODER_TICKS_PER_ROTATION = 1120;
    private double driveFactor = 1;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        dropperServo = hardwareMap.servo.get("dropper");
        planeServo = hardwareMap.servo.get("plane");
        rotateServo = hardwareMap.servo.get("rotate");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotor.setTargetPosition(5);
        dropperServo.setPosition(.3);
        planeServo.setPosition(0);
        rotateServo.setPosition(0);

        waitForStart();


        //back
        leftFront.setPower(-.25);
        leftBack.setPower(-.25);
        rightBack.setPower(.25);
        rightFront.setPower(.25);
        sleep(300);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(250);
        //left
        leftFront.setPower(-.25);
        leftBack.setPower(.25);
        rightBack.setPower(.25);
        rightFront.setPower(-.25);
        sleep(4600);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(250);
        ////right
        leftFront.setPower(.25);
        leftBack.setPower(-.25);
        rightBack.setPower(-.25);
        rightFront.setPower(.25);
        sleep(1000);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(250);
        //back
        leftFront.setPower(-.25);
        leftBack.setPower(-.25);
        rightBack.setPower(.25);
        rightFront.setPower(.25);
        sleep(3000);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(250);
        //forward
        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(-.1);
        rightBack.setPower(-.1);
        sleep(900);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(250);
        //slides
        slideMotor.setPower(.6);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1700));
        sleep(1000);
        rotateServo.setPosition(.135);
        sleep(4000);
        dropperServo.setPosition(0);
        sleep(2000);
        sleep(2000);
        slideMotor.setPower(0.6);
        slideMotor.setTargetPosition(0);
        rotateServo.setPosition(0);
        dropperServo.setPosition(.3);
        sleep(3000);
        leftFront.setPower(.5);
        leftBack.setPower(-.5);
        rightBack.setPower(-.5);
        rightFront.setPower(.5);
        sleep(1250);



    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }

}