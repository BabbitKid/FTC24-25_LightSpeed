package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;


@Autonomous(name="BlueClose", group="Robot")
@Config

public class BlueClose extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack,slideMotor;

    private Servo autoDrop;
    private Servo dropperServo, planeServo, rotateServo;
    GoBildaPinpointDriver odo;


    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";

    OpenCvWebcam webcam;
    public static double XPOSITION_1 = -5;
    public static double YPOSITION_1 = -27;
    public static double HEADING_1 = 0;
    public static double STOP = 15;
    public static double XPOSITION_2 = -5;
    public static double YPOSITION_2 = -27;
    public static double HEADING_2 = 0;
    public static double XPOSITION_3 = -5;
    public static double YPOSITION_3 = -27;
    public static double HEADING_3 = 0;
    private double ENCODER_TICKS_PER_ROTATION = 1120 * (2.0/3);





    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        autoDrop = hardwareMap.servo.get("autoDrop");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        dropperServo = hardwareMap.servo.get("dropper");
        planeServo = hardwareMap.servo.get("plane");
        rotateServo = hardwareMap.servo.get("rotate");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(188.7, 0); //Check Y
        odo.setEncoderResolution(13.26291192); // Not true https://www.revrobotics.com/rev-11-1271/


        Pose2D pos = odo.getPosition();
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);
        Thread positionUpdate = new Thread(odometry);

        positionUpdate.start();

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(.75);










        while (!opModeIsActive()) {
            //init
        }

        autoDrop.setPosition(0.625);


        waitForStart();


        if (opModeIsActive()) {
            odo.update();

                    GoToPosition.goToPosition(4, -27, 0, 3, odo, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
                    GoToPosition.goToPosition(0, -17, 0, 3, odo, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
                    GoToPosition.goToPosition(32.75, -20, -91, 3, odo, leftFront, leftBack, rightFront, rightBack, telemetry, .45);

                    leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(860));
            while (Math.abs(slideMotor.getCurrentPosition() - convertDegreesToEncoderTicks(860)) > convertDegreesToEncoderTicks(20) && opModeIsActive()) {

            }
            rotateServo.setPosition(.15);
            sleep(1000);
            dropperServo.setPosition(.0);
            sleep(1000);
            rotateServo.setPosition(.08);
            sleep(1000);
            dropperServo.setPosition(.3);
            sleep(1000);
            rotateServo.setPosition(0);
            sleep(1000);
            dropperServo.setPosition(.3);
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
            while (Math.abs(slideMotor.getCurrentPosition() - convertDegreesToEncoderTicks(0)) > convertDegreesToEncoderTicks(20) && opModeIsActive()   ) {

            }
            GoToPosition.goToPosition( 30, -8, -93, 3, odo, leftFront, leftBack, rightFront, rightBack, telemetry, .45);

        }

    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }

}

