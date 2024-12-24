package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;

import java.util.Locale;


@Autonomous(name="BlueClose", group="Robot")
@Config

public class BlueClose extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal;

    private static DcMotor leftFront, leftBack, rightFront, rightBack, slideMotor;

    private Servo intake, intakeArm, intakeSlides,fourBar, dropper, grabby;
    private static double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
    private static double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    
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
        intake = hardwareMap.servo.get("intake");
        intakeArm = hardwareMap.servo.get("intakeArm");
        intakeSlides = hardwareMap.servo.get("intakeSlides");
        fourBar = hardwareMap.servo.get("fourBar");
        dropper = hardwareMap.servo.get("dropper");
        slideMotor = hardwareMap.dcMotor.get("vertSlides");
        grabby = hardwareMap.servo.get("grabby");


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(90));


        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(188.7, 0); //Check Y
        odo.setEncoderResolution(44.9585273728); // Not true https://www.revrobotics.com/rev-11-1271/
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        Pose2D pos = odo.getPosition();


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(.75);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        odo.resetPosAndIMU();

        waitForStart();


        if (opModeIsActive()) {
            odo.update();

            GoToPositionOld.GoToPositionCall(10,10,90,.5,3, odo, leftFront, rightFront, leftBack, rightBack, telemetry);

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

        }

    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }


}

