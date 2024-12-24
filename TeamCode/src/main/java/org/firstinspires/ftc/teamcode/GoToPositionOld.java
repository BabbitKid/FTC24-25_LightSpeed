package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.Range;

import java.io.BufferedInputStream;
import java.util.Locale;

public class GoToPositionOld {

    GoBildaPinpointDriver odo;

    public static LinearOpMode activeOpMode;


    public static double DIAMETER = 15;



    public static void GoToPositionCall(
            double xTarget,
            double yTarget,
            double targetHeading,
            double speed,
            double stoppingDistance,
            GoBildaPinpointDriver odo,
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight,
            Telemetry telemetry
    ) {

        // Step 1: Set motor directions
        // Front left motor moves forward
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // Front right motor moves in reverse
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Back left motor moves forward
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // Back right motor moves in reverse
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Step 2: Initialize robot's position
        Pose2D pos = odo.getPosition();
        while (true) {
            // Update robot's position (odometry)
            odo.update();

            // Step 3: Get current position and heading
            pos = odo.getPosition();
            double currentX = pos.getX(DistanceUnit.INCH);    // Current X position in inches
            double currentY = pos.getY(DistanceUnit.INCH);    // Current Y position in inches
            double currentHeading = pos.getHeading(AngleUnit.DEGREES); // Current heading (angle) in degrees

            // Step 4: Calculate the distance to target using Pythagoras' theorem
            double dist = Math.hypot(xTarget - currentX, yTarget - currentY);

            // Step 5: Check if the robot is close enough to the target
            if (dist <= stoppingDistance) {
                break;  // Exit the loop if the robot is close enough to the target
            }

            // Step 6: Calculate velocity components in FIELD coordinates (relative to the field)
            // Determine the velocity along the X axis towards the target (field-relative)
            double vxField = speed * (xTarget - currentX) / dist;
            // Determine the velocity along the Y axis towards the target (field-relative)
            double vyField = speed * (yTarget - currentY) / dist;

            // Step 7: Convert velocity from FIELD coordinates to ROBOT coordinates
            // Get the robot's heading in radians (for rotation adjustments)
            double theta = Math.toRadians(currentHeading);

            // Calculate the velocity along the robot's X axis (robot-relative)
            // Rotate the field velocity by the robot's heading using trigonometric functions
            double vxRobot = vxField * Math.cos(theta) + vyField * Math.sin(theta);

            // Calculate the velocity along the robot's Y axis (robot-relative)
            // Rotate the field velocity by the robot's heading using trigonometric functions
            double vyRobot = -vxField * Math.sin(theta) + vyField * Math.cos(theta);

            // Step 8: Calculate the heading error to correct the robot's rotation
            double headingError = targetHeading - currentHeading;
            // Normalize heading error to the range [-180, 180]
            headingError = ((headingError + 180) % 360) - 180;
            // Apply proportional control to adjust rotational speed
            double kP = 0.01;  // Proportional gain constant (this value might need to be tuned)
            double rotationalSpeed = headingError * kP; // Adjust rotational speed based on heading error

            // Step 9: Calculate the power for each motor
            // Combine linear movement (vxRobot, vyRobot) and rotational movement (rotationalSpeed)
            // Front Left motor power: forward movement (vy) + strafe (vx) + rotational correction
            double flPower = vyRobot + vxRobot + rotationalSpeed;
            // Front Right motor power: forward movement (vy) - strafe (vx) - rotational correction
            double frPower = vyRobot - vxRobot - rotationalSpeed;
            // Back Left motor power: forward movement (vy) - strafe (vx) + rotational correction
            double blPower = vyRobot - vxRobot + rotationalSpeed;
            // Back Right motor power: forward movement (vy) + strafe (vx) - rotational correction
            double brPower = vyRobot + vxRobot - rotationalSpeed;

            // Step 10: Clip the motor powers to be within the valid range [-1.0, 1.0]
            // Ensures that motor power does not exceed the max and min values
            flPower = Range.clip(flPower, -1.0, 1.0);
            frPower = Range.clip(frPower, -1.0, 1.0);
            blPower = Range.clip(blPower, -1.0, 1.0);
            brPower = Range.clip(brPower, -1.0, 1.0);

            // Step 11: Set the motor power values to the calculated values
            // Apply the power to the motors to move the robot
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);

            // Step 12: Telemetry for debugging and monitoring
            // Display the target position and current position of the robot
            telemetry.addData("Target", "X: %.2f, Y: %.2f, Heading: %.2f", xTarget, yTarget, targetHeading);
            // Display the current position and heading of the robot
            telemetry.addData("Current", "X: %.2f, Y: %.2f, Heading: %.2f", currentX, currentY, currentHeading);
            // Display the distance to the target
            telemetry.addData("Distance", dist);
            // Display motor power values for each motor
            telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", flPower, frPower, blPower, brPower);
            // Display the velocity components along the robot's axes
            telemetry.addData("vxRobot", vxRobot);
            telemetry.addData("vyRobot", vyRobot);
            // Display the rotational correction applied
            telemetry.addData("Rotational", rotationalSpeed);
            // Update the telemetry display
            telemetry.update();
        }

        // Step 13: Stop the robot after reaching the target position
        // Set motor powers to zero to stop the robot
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}
