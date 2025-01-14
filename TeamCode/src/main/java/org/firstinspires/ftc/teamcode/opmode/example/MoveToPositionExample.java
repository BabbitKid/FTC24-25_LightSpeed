package org.firstinspires.ftc.teamcode.opmode.example;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name = "Go to Position Example", group = "Examples")
public class MoveToPositionExample extends OpMode {

    // Declare a Follower to handle path following
    private Follower follower;

    // Declare the motors for the robot's left and right drive wheels
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // Define the starting pose (robot's initial position and orientation)
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));  // Start at (x=0, y=0) with a 0-degree heading

    // Define the target pose (position and orientation the robot should move to)
    // Move 24 inches in both X and Y directions and 90 degrees heading
    private final Pose targetPose = new Pose(24, 24, Math.toDegrees(90));  // Target at (x=24, y=24) with a 90-degree heading

    // Path to follow
    private Path moveToPosition;

    @Override
    public void init() {
        // Initialize motors from the hardware map (motors must be configured in the robot controller)
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Reverse the right motor if needed (depending on motor configuration)
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize the follower object, which handles path following logic
        follower = new Follower(hardwareMap);

        // Set the initial pose for the follower (the robot's starting position)
        follower.setStartingPose(startPose);

        // Build the path from the startPose to the targetPose
        buildPath();
    }

    // Method to build the path from the startPose to the targetPose
    public void buildPath() {
        // Create a Bezier line (smooth curve) between the start and target poses
        moveToPosition = new Path(new BezierLine(new Point(startPose), new Point(targetPose)));

        // Set the heading interpolation for the path so that the robot smoothly transitions its heading
        moveToPosition.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());
    }

    @Override
    public void loop() {
        // Update the follower object each loop iteration, checking the robot's progress along the path
        follower.update();

        // Follow the path from startPose to targetPose
        follower.followPath(moveToPosition);

        // Variables to store the motor power for left and right wheels
        double leftPower = 0.0;
        double rightPower = 0.0;

        // If the follower is still "busy" (robot still moving towards the target), set motor power
        if (follower.isBusy()) {
            leftPower = 0.5;  // Example power for the left motor
            rightPower = 0.5; // Example power for the right motor
        }

        // Apply the calculated power to the motors to move the robot
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Provide real-time telemetry data to the Driver Hub for monitoring
        telemetry.addData("x", follower.getPose().getX()); // Current x position
        telemetry.addData("y", follower.getPose().getY()); // Current y position
        telemetry.addData("heading", follower.getPose().getHeading()); // Current heading
        telemetry.update(); // Update the telemetry display
    }

    @Override
    public void start() {
        // This method is called when the autonomous period starts
        // It's usually used for resetting timers or initializing states
    }

    @Override
    public void stop() {
        // This method is called when the autonomous period ends
        // It stops the motors to prevent any unwanted movement after the task is finished
        leftDrive.setPower(0);  // Stop the left motor
        rightDrive.setPower(0); // Stop the right motor
    }

    @Override
    public void init_loop() {
        // This method is called during the initialization phase while waiting for "Play" to be pressed
        // It is not usually needed unless there's specific setup that needs to be done continuously
    }
}
