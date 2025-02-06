package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTest", group="Linear Opmode")
public class SingleMotorTest extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private static DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                leftFront.setPower(1);
                telemetry.addData("leftFrontMove", leftFront.getCurrentPosition());
            }
            if (gamepad1.b){
                leftBack.setPower(1);
                telemetry.addData("leftBackMove", leftBack.getCurrentPosition());
            }
            if (gamepad1.y){
                rightFront.setPower(1);
                telemetry.addData("rightFrontMove", rightFront.getCurrentPosition());
            }
            if (gamepad1.x){
                rightBack.setPower(1);
                telemetry.addData("rightBackMove", rightBack.getCurrentPosition());
            }

        }
    }
}