package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "TestAuto")
public class TestAuto extends OpMode {

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
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    //my new pose
    private final Pose goOut1Pose = new Pose(50, 1, Math.toRadians(0));

    private final Pose getBlock1Pose = new Pose(50, -4, Math.toRadians(0));

    private final Pose goIn1Pose = new Pose(10, -4, Math.toRadians(0));

    private final Pose goOut2Pose = new Pose(50, -4, Math.toRadians(0));

    private final Pose getBlock2Pose = new Pose(50, -8, Math.toRadians(0));

    private final Pose goIn2Pose = new Pose(10, -8, Math.toRadians(0));

    private final Pose goOut3Pose = new Pose(50, -8, Math.toRadians(0));

    private final Pose getBlock3Pose = new Pose(50, -12, Math.toRadians(0));

    private final Pose goIn3Pose = new Pose(10, -12, Math.toRadians(0));


    private Path scorePreload, park;
    private PathChain goOut1, goToBlock1, goIn1, goOut2, goToBlock2, goIn2, goOut3, goToBlock3, goIn3, getClip1, goClip1;


    public void buildPaths() {

        follower.setMaxPower(.5);


        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(goOut1Pose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), goOut1Pose.getHeading());

        goOut1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(goOut1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goOut1Pose.getHeading())
                .build();

        goToBlock1 = follower.pathBuilder()
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

        goToBlock2 = follower.pathBuilder()
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

        goToBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goOut3Pose), new Point(getBlock3Pose)))
                .setLinearHeadingInterpolation(goOut3Pose.getHeading(), getBlock3Pose.getHeading())
                .build();

        goIn3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(getBlock3Pose), new Point(goIn3Pose)))
                .setLinearHeadingInterpolation(getBlock3Pose.getHeading(), goIn3Pose.getHeading())
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:


                if(follower.getPose().getX() > (goOut1Pose.getX() - 1) && follower.getPose().getY() > (goOut1Pose.getY() - 1)) {

                    follower.followPath(goOut1,true);
                    setPathState(2);
                }
                break;
            case 2:


                if(follower.getPose().getX() > (getBlock1Pose.getX() - 1) && follower.getPose().getY() > (getBlock1Pose.getY() - 1)) {

                    follower.followPath(goToBlock1,true);
                    setPathState(3);
                }
                break;
            case 3:


                if(follower.getPose().getX() > (goIn1Pose.getX() - 1) && follower.getPose().getY() > (goIn1Pose.getY() - 1)) {

                    follower.followPath(goIn1,true);
                    setPathState(4);
                }
                break;
            case 4:


                if(follower.getPose().getX() > (goOut2Pose.getX() - 1) && follower.getPose().getY() > (goOut2Pose.getY() - 1)) {

                    follower.followPath(goOut2,true);
                    setPathState(5);
                }
                break;
            case 5:


                if(follower.getPose().getX() > (getBlock2Pose.getX() - 1) && follower.getPose().getY() > (getBlock2Pose.getY() - 1)) {

                    follower.followPath(goToBlock2,true);
                    setPathState(6);
                }
                break;
            case 6:


                if(follower.getPose().getX() > (goIn2Pose.getX() - 1) && follower.getPose().getY() > (goIn2Pose.getY() - 1)) {

                    follower.followPath(goIn2,true);
                    setPathState(7);
                }
                break;
            case 7:


                if(follower.getPose().getX() > (goOut3Pose.getX() - 1) && follower.getPose().getY() > (goOut3Pose.getY() - 1)) {

                    follower.followPath(goOut3,true);
                    setPathState(8);
                }
                break;
            case 8:


                if(follower.getPose().getX() > (getBlock3Pose.getX() - 1) && follower.getPose().getY() > (getBlock3Pose.getY() - 1)) {

                    follower.followPath(goToBlock3,true);
                    setPathState(9);
                }
                break;
            case 9:


                if(follower.getPose().getX() > (goIn3Pose.getX() - 1) && follower.getPose().getY() > (goIn3Pose.getY() - 1)) {

                    follower.followPath(goIn3,true);
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
        autonomousPathUpdate();

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

        // Set the claw to positions for init

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
