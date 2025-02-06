package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "ClippingAuto")
public class ClippingAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
    public ClawSubsystem claw;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */

    private static DcMotor leftFront, leftBack, rightFront, rightBack, intakeMotor, rightSlidesMotor, leftSlidesMotor;

    private Servo armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;

    private final Pose startPose = new Pose(0, -12, Math.toRadians(0));

    private final Pose skibidiPose = new Pose(-26, -46, Math.toRadians(0));

    private final Pose goOut1Pose = new Pose(-50, -13, Math.toRadians(0));

    private final Pose getBlock1Pose = new Pose(-50, -1, Math.toRadians(0));

    private final Pose goIn1Pose = new Pose(-4, -3, Math.toRadians(0));

    private final Pose goOut2Pose = new Pose(-50, -3, Math.toRadians(0));

    private final Pose getBlock2Pose = new Pose(-50, 9, Math.toRadians(0));

    private final Pose goIn2Pose = new Pose(-4, 8, Math.toRadians(0));

    private final Pose goOut3Pose = new Pose(-50, 8, Math.toRadians(0));

    private final Pose getBlock3Pose = new Pose(-50, 11.85, Math.toRadians(0));

    private final Pose goIn3Pose = new Pose(-4, 9.5, Math.toRadians(0));

    private final Pose getClip1Pose = new Pose(-16, -13, Math.toRadians(0));

    private final Pose goClip1Pose = new Pose(-26, -40, Math.toRadians(0));

    private final Pose getClip2Pose = new Pose(-16, -13, Math.toRadians(0));

    private final Pose goClip2Pose = new Pose(-26, -42, Math.toRadians(0));

    private final Pose getClip3Pose = new Pose(-16, -13, Math.toRadians(0));

    private final Pose goClip3Pose = new Pose(-26, -48, Math.toRadians(0));

    private final Pose parkPose = new Pose(-26, -48, Math.toRadians(0));

    private final Pose parkControlPose = new Pose(-26, -48, Math.toRadians(0));
    private Path scorePreload, park;
    private PathChain skibidi, goOut1, getBlock1, goIn1, goOut2, getBlock2, goIn2, goOut3, getBlock3, goIn3, getClip1, goClip1, getClip2, goClip2, getClip3, goClip3;

    public static boolean checkWithinOneInch(double currentX, double targetX, double currentY, double targetY) {
        double distance = Math.sqrt(Math.pow(currentX - targetX, 2) + Math.pow(currentY - targetY, 2));
        return distance <= 1.0;
    }




    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(startPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());


        skibidi = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(skibidiPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), skibidiPose.getHeading())
                .build();

        goOut1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(goOut1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goOut1Pose.getHeading())
                .build();

        getBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goOut1Pose), new Point(getBlock1Pose)))
                .setLinearHeadingInterpolation(goOut1Pose.getHeading(), getBlock1Pose.getHeading())
                .build();

        goIn1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock1Pose), new Point(goIn1Pose)))
                .setLinearHeadingInterpolation(getBlock1Pose.getHeading(), goIn1Pose.getHeading())
                .build();

        goOut2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goIn1Pose), new Point(goOut2Pose)))
                .setLinearHeadingInterpolation(goIn1Pose.getHeading(), goOut2Pose.getHeading())
                .build();

        getBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goOut2Pose), new Point(getBlock2Pose)))
                .setLinearHeadingInterpolation(goOut2Pose.getHeading(), getBlock2Pose.getHeading())
                .build();

        goIn2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock2Pose), new Point(goIn2Pose)))
                .setLinearHeadingInterpolation(getBlock2Pose.getHeading(), goIn2Pose.getHeading())
                .build();

        goOut3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goIn2Pose), new Point(goOut3Pose)))
                .setLinearHeadingInterpolation(goIn2Pose.getHeading(), goOut3Pose.getHeading())
                .build();

        getBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goOut3Pose), new Point(getBlock3Pose)))
                .setLinearHeadingInterpolation(goOut3Pose.getHeading(), getBlock3Pose.getHeading())
                .build();

        goIn3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock3Pose), new Point(goIn3Pose)))
                .setLinearHeadingInterpolation(getBlock3Pose.getHeading(), goIn3Pose.getHeading())
                .build();

        getClip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goIn3Pose), new Point(getClip1Pose)))
                .setLinearHeadingInterpolation(goIn3Pose.getHeading(), getClip1Pose.getHeading())
                .build();

        goClip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getClip1Pose), new Point(goClip1Pose)))
                .setLinearHeadingInterpolation(getClip1Pose.getHeading(), goClip1Pose.getHeading())
                .build();

        getClip2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goClip1Pose), new Point(getClip2Pose)))
                .setLinearHeadingInterpolation(goClip1Pose.getHeading(), getClip2Pose.getHeading())
                .build();

        goClip2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getClip2Pose), new Point(goClip2Pose)))
                .setLinearHeadingInterpolation(getClip2Pose.getHeading(), goClip2Pose.getHeading())
                .build();

        getClip3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goClip2Pose), new Point(getClip3Pose)))
                .setLinearHeadingInterpolation(goClip2Pose.getHeading(), getClip3Pose.getHeading())
                .build();

        goClip3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getClip3Pose), new Point(goClip3Pose)))
                .setLinearHeadingInterpolation(getClip3Pose.getHeading(), goClip3Pose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(getClip3Pose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(getClip3Pose.getHeading(), parkPose.getHeading());





    }

    public void autonomousPathUpdate() {

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
        rightSlidesMotor.setTargetPosition(-20);
        rightSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlidesMotor.setTargetPosition(-20);
        leftSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlidesMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightSlideArm.setDirection(Servo.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);



        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (checkWithinOneInch(follower.getPose().getX(), skibidiPose.getX(), follower.getPose().getY(), skibidiPose.getY())) {
                    setPathState(2);
                } else {
                    leftSlideArm.setPosition(.25);
                    rightSlideArm.setPosition(.25);
                    armRotate.setPosition(.54);
                    rightSlidesMotor.setTargetPosition(-5);
                    leftSlidesMotor.setTargetPosition(-20);
                    follower.followPath(skibidi, true);

                }
                break;
            case 2:


                if (checkWithinOneInch(follower.getPose().getX(), goOut1Pose.getX(), follower.getPose().getY(), goOut1Pose.getY())) {
                    setPathState(3);
                } else {
                    follower.followPath(goOut1, true);

                }
                break;
            case 3:



                if (checkWithinOneInch(follower.getPose().getX(), getBlock1Pose.getX(), follower.getPose().getY(), getBlock1Pose.getY())) {
                    setPathState(4);
                    break;
                } else {
                    follower.followPath(getBlock1, true);

                }
                break;
            case 4:

                if (checkWithinOneInch(follower.getPose().getX(), goIn1Pose.getX(), follower.getPose().getY(), goIn1Pose.getY())) {
                    setPathState(5);
                    break;
                } else {
                    follower.followPath(goIn1, true);

                }
                break;
            case 5:

                if (checkWithinOneInch(follower.getPose().getX(), goOut2Pose.getX(), follower.getPose().getY(), goOut2Pose.getY())) {
                    setPathState(6);
                    break;
                } else {
                    follower.followPath(goOut2, true);

                }
                break;
            case 6:

                if (checkWithinOneInch(follower.getPose().getX(), getBlock2Pose.getX(), follower.getPose().getY(), getBlock2Pose.getY())) {
                    setPathState(7);
                    break;
                } else {
                    follower.followPath(getBlock2, true);

                }
                break;
            case 7:

                if (checkWithinOneInch(follower.getPose().getX(), goIn2Pose.getX(), follower.getPose().getY(), goIn2Pose.getY())) {
                    setPathState(8);
                    break;
                } else {
                    follower.followPath(goIn2, true);

                }

                break;
            case 8:

                if (checkWithinOneInch(follower.getPose().getX(), goOut3Pose.getX(), follower.getPose().getY(), goOut3Pose.getY())) {
                    setPathState(9);
                    break;
                } else {
                    follower.followPath(goOut3, true);

                }

                break;
            case 9:

                if (checkWithinOneInch(follower.getPose().getX(), getBlock3Pose.getX(), follower.getPose().getY(), getBlock3Pose.getY())) {
                    setPathState(10);
                    break;
                } else {
                    follower.followPath(getBlock3, true);

                }

                break;
            case 10:

                if (checkWithinOneInch(follower.getPose().getX(), goIn3Pose.getX(), follower.getPose().getY(), goIn3Pose.getY())) {
                    setPathState(11);
                    break;
                } else {
                    follower.followPath(goIn3, true);

                }

                break;
            case 11:

                if (checkWithinOneInch(follower.getPose().getX(), getClip1Pose.getX(), follower.getPose().getY(), getClip1Pose.getY())) {
                    setPathState(12);
                    break;
                } else {
                    follower.followPath(getClip1, true);


                }

                break;
            case 12:

                if (checkWithinOneInch(follower.getPose().getX(), goClip1Pose.getX(), follower.getPose().getY(), goClip1Pose.getY())) {
                    setPathState(13);
                    break;
                } else {
                    follower.followPath(goClip1, true);

                }

                break;
            case 13:

                if (checkWithinOneInch(follower.getPose().getX(), getClip2Pose.getX(), follower.getPose().getY(), getClip2Pose.getY())) {
                    setPathState(14);
                    break;
                } else {
                    follower.followPath(getClip2, true);

                }

                break;
            case 14:

                if (checkWithinOneInch(follower.getPose().getX(), goClip2Pose.getX(), follower.getPose().getY(), goClip2Pose.getY())) {
                    setPathState(15);
                    break;
                } else {
                    follower.followPath(goClip2, true);

                }

                break;
            case 15:

                if (checkWithinOneInch(follower.getPose().getX(), getClip3Pose.getX(), follower.getPose().getY(), getClip3Pose.getY())) {
                    setPathState(16);
                    break;
                } else {
                    follower.followPath(getClip3, true);

                }

                break;
            case 16:

                if (checkWithinOneInch(follower.getPose().getX(), goClip3Pose.getX(), follower.getPose().getY(), goClip3Pose.getY())) {
                    setPathState(17);
                    break;
                } else {
                    follower.followPath(goClip3, true);

                }

                break;
            case 17:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(checkWithinOneInch(follower.getPose().getX(), parkPose.getX(), follower.getPose().getY(), parkPose.getY())) {
                    setPathState(-1);
                    break;
                } else {
                    follower.followPath(goClip3, true);
                }
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void loop() {


        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        claw.linearSlidesIn();
        claw.intakeDown();


        if (checkWithinOneInch(follower.getPose().getX(), goOut1Pose.getX(), follower.getPose().getY(), goOut1Pose.getY())) {
            telemetry.addData("Working", pathState);
        } else {
            telemetry.addData("Not Working", pathState);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        claw = new ClawSubsystem(hardwareMap);


    }


    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void stop() {
    }
}
