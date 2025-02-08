package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
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

@Autonomous(name = "NewClipping")
public class NewClipping extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    public ClawSubsystem claw;

    private static DcMotor leftFront, leftBack, rightFront, rightBack, intakeMotor, rightSlidesMotor, leftSlidesMotor;
    private Servo armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;
    private final Pose startPose = new Pose(0, -12, Math.toRadians(0));
    private final Pose rotateGrabbyPose = new Pose(0, -12, 0);
    private final Pose clipReadyPose = new Pose(-26, -46, Math.toRadians(0));
    private final Pose clipPose = new Pose(-26.1, -46, Math.toRadians(0));
    private final Pose driveAfterClipPose = new Pose(-20, -46, Math.toRadians(0));
    private final Pose openGrabbyPose = new Pose(-20, -46, Math.toRadians(0));
    private final Pose armDownAfterClipPose = new Pose(-20, -46, Math.toRadians(0));


    private Path scorePreload, park;
    private PathChain rotateGrabby, clipReady, clip, driveAfterClip, openGrabby, armDownAfterClip;

    public static boolean checkWithinOneInch(double currentX, double targetX, double currentY, double targetY) {
        double distance = Math.sqrt(Math.pow(currentX - targetX, 2) + Math.pow(currentY - targetY, 2));
        return distance <= 1.0;
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(startPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading());

        rotateGrabby = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(rotateGrabbyPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotateGrabbyPose.getHeading())
                .build();


        clipReady = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rotateGrabbyPose), new Point(clipReadyPose)))
                .setLinearHeadingInterpolation(rotateGrabbyPose.getHeading(), clipReadyPose.getHeading())
                .build();

        clip = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipReadyPose), new Point(clipPose)))
                .setLinearHeadingInterpolation(clipReadyPose.getHeading(), clipPose.getHeading())
                .build();

        driveAfterClip = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipPose), new Point(driveAfterClipPose)))
                .setLinearHeadingInterpolation(clipPose.getHeading(), driveAfterClipPose.getHeading())
                .build();

        openGrabby = follower.pathBuilder()
                .addPath(new BezierLine(new Point(driveAfterClipPose), new Point(openGrabbyPose)))
                .setLinearHeadingInterpolation(driveAfterClipPose.getHeading(), openGrabbyPose.getHeading())
                .build();

        armDownAfterClip = follower.pathBuilder()
                .addPath(new BezierLine(new Point(driveAfterClipPose), new Point(armDownAfterClipPose)))
                .setLinearHeadingInterpolation(driveAfterClipPose.getHeading(), armDownAfterClipPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() throws InterruptedException {
        rightSlidesMotor = hardwareMap.dcMotor.get("rightSlidesMotor");




        switch (pathState) {
            case 0:
                if (checkWithinOneInch(follower.getPose().getX(), startPose.getX(), follower.getPose().getY(), startPose.getY())) {
                    setPathState(1);
                } else {
                    follower.followPath(scorePreload, true);
                }
                break;
           /** case 1:
                if (!follower.isBusy()) {
                    claw.grabbyClose();
                    claw.armRotateGrabby();
                    follower.followPath(clipReady, true);
                    setPathState(2);
                } */
            case 1:
                if (!follower.isBusy()) {
                    claw.grabbyClose();
                    claw.readyClip();
                    follower.followPath(clipReady, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    claw.grabbyClose();
                    claw.clip();
                    claw.slidesUpForClip();
                    follower.followPath(clip, true);
                    if (!claw.slidesUpForClip() == false) {
                        boolean holdingPosition = true;
                    } else
                        setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    claw.clip();
                    claw.slidesUpForClip();
                    follower.followPath(driveAfterClip, true);
                    claw.grabbyClose();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    claw.grabbyOpen();
                    follower.followPath(openGrabby, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    claw.grabbyOpen();
                    claw.getClip();
                    claw.slidesDown();
                    follower.followPath(armDownAfterClip, true);
                    setPathState(6);
                }
                break;

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
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        claw.linearSlidesIn();
        claw.intakeDown();




        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("SlidesPos", rightSlidesMotor.getCurrentPosition());
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