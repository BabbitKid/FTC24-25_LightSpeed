package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;




@Autonomous(name="Test Test", group="Robot")
@Config

public class GoToPositionTest extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack;

    private Servo autoDrop;


    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";

    OpenCvWebcam webcam;
    public static double STOP = 3;
    public static double XPOSITION_1 = 0;
    public static double YPOSITION_1 = 0;
    public static double HEADING_1 = 0;
    public static double XPOSITION_2 = 0;
    public static double YPOSITION_2 = 0;
    public static double HEADING_2 = 0;
    public static double XPOSITION_3 = 0;
    public static double YPOSITION_3 = 0;
    public static double HEADING_3 = 0;
    public static double XPOSITION_4 = 0;
    public static double YPOSITION_4 = 0;
    public static double HEADING_4 = 0;
    public static double XPOSITION_5 = 0;
    public static double YPOSITION_5 = 0;
    public static double HEADING_5 = 0;
    public static double XPOSITION_6 = 0;
    public static double YPOSITION_6 = 0;
    public static double HEADING_6 = 0;
    public static double XPOSITION_7 = 0;
    public static double YPOSITION_7 = 0;
    public static double HEADING_7 = 0;
    public static double XPOSITION_8 = 0;
    public static double YPOSITION_8 = 0;
    public static double HEADING_8 = 0;
    public static double XPOSITION_9 = 0;
    public static double YPOSITION_9 = 0;
    public static double HEADING_9 = 0;
    public static double XPOSITION_10 = 0;
    public static double YPOSITION_10 = 0;
    public static double HEADING_10 = 0;
    public static double XPOSITION_11 = 0;
    public static double YPOSITION_11 = 0;
    public static double HEADING_11 = 0;
    public static double XPOSITION_12 = 0;
    public static double YPOSITION_12 = 0;
    public static double HEADING_12 = 0;






    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        autoDrop = hardwareMap.servo.get("autoDrop");



        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        /*imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();*/
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);
        Thread positionUpdate = new Thread(odometry);

        positionUpdate.start();

        /*

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RedConeDetectionPipeline samplePipeline = new RedConeDetectionPipeline(telemetry);
        webcam.setPipeline(samplePipeline);


        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        int location = 0;


        while (!opModeIsActive()) {
            location = samplePipeline.getLocation();
            if (location == 0) {
                telemetry.addData("Left",0);
            } if (location == 1) {
                telemetry.addData("Center", 1);
            } if (location == 2) {
                telemetry.addData("Right", 2);
            }
            telemetry.update();
        }*/

        autoDrop.setPosition(0.625);


        waitForStart();


        if (opModeIsActive()) {

                /*GoToPosition.goToPosition(0, -46,0,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .8);
                GoToPosition.goToPosition(0, -46,190,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .8);*/
            GoToPosition.goToPosition(XPOSITION_1, YPOSITION_1, HEADING_1, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_2, YPOSITION_2, HEADING_2, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            autoDrop.setPosition(.4);
            GoToPosition.goToPosition(XPOSITION_3, YPOSITION_3, HEADING_3, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_4, YPOSITION_4, HEADING_4, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_5, YPOSITION_5, HEADING_5, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_6, YPOSITION_6, HEADING_6, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_7, YPOSITION_7, HEADING_7, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_8, YPOSITION_8, HEADING_8, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_9, YPOSITION_9, HEADING_9, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_10, YPOSITION_10, HEADING_10, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_11, YPOSITION_11, HEADING_11, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);
            GoToPosition.goToPosition(XPOSITION_12, YPOSITION_12, HEADING_12, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .45);


            /*GoToPosition.goToPosition(24, 48,90,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 1);
            GoToPosition.goToPosition(0,0,0, 15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 1);
            sleep(5000);*/

        }

    }

}

