package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.bucketScoreSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
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

@Autonomous(name = "BucketAuto")
public class BucketAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    public bucketScoreSubsystem bucket;
    public ClawSubsystem claw;

    private static DcMotor leftFront, leftBack, rightFront, rightBack, intakeMotor, rightSlidesMotor, leftSlidesMotor;
    private Servo armRotate, leftIntake, rightIntake, linearSlides, grabby, rightSlideArm, leftSlideArm;


    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */


    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private final Pose scoreBucketPose = new Pose(2, 24, Math.toRadians(-45));

    private final Pose scoreFinalBucketPose = new Pose(15, 19, Math.toRadians(-45));

    private final Pose getBlock1Pose = new Pose(36, 12, Math.toRadians(0));

    private final Pose getBlock2Pose = new Pose(36, 25, Math.toRadians(0));

    private final Pose getBlock3Pose = new Pose(40, 19, Math.toRadians(90));

    private final Pose alignStraight1 = new Pose (12,12, Math.toRadians(0));

    private final Pose alignStraight2 = new Pose(12, 25);


    private Path scorePreload, park;
    private PathChain scoreBucketPre, alignStraightPath, getBlock1, scoreBucket1, alignStraightPath2, getBlock2, scoreBucket2, getBlock3, scoreFinalBucket;

    public static boolean checkWithinOneInch(double currentX, double targetX, double currentY, double targetY) {
        // Check if within one inch
        double distance = Math.sqrt(Math.pow(currentX - targetX, 2) + Math.pow(currentY - targetY, 2));
        boolean withinOneInch = distance <= 1.5;
        // Return true only if both checks pass
        return withinOneInch;
    }

    public static boolean checkHeading(double currentHeading, double targetHeading, double headingToleranceDegrees) {
        // Convert degree tolerance to radians for comparison
        double headingTolerance = Math.toRadians(headingToleranceDegrees);

        // Check if within the heading tolerance
        double angularDifference = Math.abs(currentHeading - targetHeading);
        // Normalize angular difference to be within [0, PI]
        angularDifference = angularDifference % (2 * Math.PI);
        if (angularDifference > Math.PI) {
            angularDifference = 2 * Math.PI - angularDifference;
        }
        boolean withinHeadingTolerance = angularDifference <= headingTolerance;

        // Return true only if both checks pass
        return withinHeadingTolerance;
    }



    public void buildPaths() {




        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreBucketPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreBucketPose.getHeading());

        scoreBucketPre = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scoreBucketPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreBucketPose.getHeading())
             //   .setPathEndTimeoutConstraint(11) // Timeout for the entire chain
                .build();

        alignStraightPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreBucketPose), new Point(alignStraight1)))
                .setLinearHeadingInterpolation(scoreBucketPose.getHeading(), alignStraight1.getHeading())
          //      .setPathEndTimeoutConstraint(11) // Timeout for the entire chain
                .build();

        getBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(alignStraight1), new Point(getBlock1Pose)))
                .setLinearHeadingInterpolation(alignStraight1.getHeading(), getBlock1Pose.getHeading())
                .build();

        scoreBucket1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock1Pose), new Point(scoreBucketPose)))
                .setLinearHeadingInterpolation(getBlock1Pose.getHeading(), scoreBucketPose.getHeading())
                .build();
        alignStraightPath2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreBucketPose), new Point(alignStraight2)))
                .setLinearHeadingInterpolation(scoreBucketPose.getHeading(), alignStraight2.getHeading())
                .build();

        getBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(alignStraight2), new Point(getBlock2Pose)))
                .setLinearHeadingInterpolation(alignStraight2.getHeading(), getBlock2Pose.getHeading())
                .build();

        scoreBucket2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock2Pose), new Point(scoreBucketPose)))
                .setLinearHeadingInterpolation(getBlock2Pose.getHeading(), scoreBucketPose.getHeading())
                .build();

        getBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreBucketPose), new Point(getBlock3Pose)))
                .setLinearHeadingInterpolation(scoreBucketPose.getHeading(), getBlock3Pose.getHeading())
                .build();

        scoreFinalBucket = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock3Pose), new Point(scoreFinalBucketPose)))
                .setLinearHeadingInterpolation(getBlock3Pose.getHeading(), scoreFinalBucketPose.getHeading())
                .build();


    }

    public void autonomousPathUpdate() throws InterruptedException {
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        rightIntake.setDirection(Servo.Direction.REVERSE);
        linearSlides = hardwareMap.servo.get("linearSlides");


        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()){
                    bucket.slidesToBucket();
                    follower.followPath(scoreBucketPre, true);
              //      public void holdPoint(scoreBucketPre, double heading);
                if (!bucket.slidesToBucket() == false) {
                    boolean holdingPosition = true;
                } else
                    setPathState(2);
                }

                break;

            case 2:
                if (!follower.isBusy()) {
                    bucket.slidesDown1();
                    follower.followPath(alignStraightPath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    bucket.intakeRotate();
                    follower.followPath(getBlock1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    bucket.grabberandintake();
                    follower.followPath(scoreBucket1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    if (!bucket.slidesToBucket() == false) {
                        boolean holdingPosition = true;
                    } else
                    follower.followPath(alignStraightPath2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    bucket.slidesDown1();
                    bucket.intakeRotate();
                    follower.followPath(getBlock2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    Thread.sleep(1000);
                    bucket.grabberandintake();
                    follower.followPath(scoreBucket2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    if (!bucket.slidesToBucket() == false) {
                        boolean holdingPosition = true;
                    } else
                    follower.followPath(getBlock3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    bucket.slidesDown1();
                    follower.followPath(scoreFinalBucket, true);
                    setPathState(10);
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

        if (checkHeading(follower.getPose().getHeading(), getBlock1Pose.getHeading(), 7)){
            telemetry.addData("Working", pathState);
        } else {
            telemetry.addData("Not Working", pathState);
        }

        /**


         if (checkWithinOneInch(follower.getPose().getX(), goOut1Pose.getX(), follower.getPose().getY(), goOut1Pose.getY())) {
            telemetry.addData("Working", pathState);
        } else {
            telemetry.addData("Not Working", pathState);
        }


         **/

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        leftIntake.setPosition(.45);
        rightIntake.setPosition(.45);
        linearSlides.setPosition(.75);
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
        bucket = new bucketScoreSubsystem(hardwareMap);

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
