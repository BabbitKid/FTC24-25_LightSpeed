package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedInputStream;

public class GoToPosition {

    public static double DIAMETER = 15;

    public static void goToPosition(double x, double y, double heading, double stoppingDistance,GoBildaPinpointDriver odo, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, Telemetry t, double power) {
        //stop

        long start = System.currentTimeMillis();
//       double currentX = odometry.getXCoordinate();
//        double currentY = odometry.getYCoordinate();
//        double currentHeading = odometry.getHeading();
        Pose2D pos = odo.getPosition();
        double currentX = pos.getX(DistanceUnit.INCH);
        double currentY = pos.getY(DistanceUnit.INCH);
        double currentHeading = pos.getHeading(AngleUnit.DEGREES);

        double differenceInX = x - currentX;
        double differenceInY = y - currentY;
        double differenceInHeading = heading - currentHeading;

        double outputTheta = differenceInHeading / 360.0 * DIAMETER * Math.PI;
        double h = Math.pow(Math.pow(differenceInX, 2) + Math.pow(differenceInY, 2), 1.0 / 2);
        double totalDistance = h + Math.abs(outputTheta);
        while (totalDistance > .7 && System.currentTimeMillis()-start < 5000) {
            t.addData("TD", totalDistance);
            odo.update();
            pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.INCH);
            currentY = pos.getY(DistanceUnit.INCH);
            currentHeading = pos.getHeading(AngleUnit.DEGREES);

            //find the difference in the x and y
            differenceInX = x - currentX;
            differenceInY = y - currentY;
            differenceInHeading = heading - currentHeading;

            //find the h by using the pythagorean theorem
            //differenceInX squared + differenceInY squared = h squared
            h = Math.pow(Math.pow(differenceInX, 2) + Math.pow(differenceInY, 2), 1.0 / 2);
            t.addData("difInX",differenceInX);
            t.addData("difInY", differenceInY);
            totalDistance = h + Math.abs(outputTheta);

            //find theta val
            double thetaP = Math.toDegrees(Math.atan2(differenceInX, differenceInY));
            double theta = -currentHeading + thetaP;
            t.addData("thetaP", thetaP);
            t.addData("heading", heading);

            //get the outputs
            double outputX = Math.sin(Math.toRadians(theta)) * h;
            double outputY = Math.cos(Math.toRadians(theta)) * h;
            outputTheta = differenceInHeading / 360.0 * DIAMETER * Math.PI;

            //calculate the motor powers
            double theNumberYouNeedInTheMatrix = Math.pow(2, 1.0 / 2) / 2;
            double leftFrontPower = outputX + (outputY * theNumberYouNeedInTheMatrix) + outputTheta;
            double leftBackPower = -outputX + (outputY * theNumberYouNeedInTheMatrix) + outputTheta;
            double rightFrontPower = -outputX + (outputY * theNumberYouNeedInTheMatrix) + (-outputTheta);
            double rightBackPower = outputX + (outputY * theNumberYouNeedInTheMatrix) + (-outputTheta);
            t.addData("oX", outputX);
            t.addData("oY", outputY);
            t.addData("oT", outputTheta);


            //Create a list of the motor powers
            double[] motorOutputs = new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};
            //normalize the matrix
            double maxOutput = 0;
            for (int i = 0; i < 4; i++) {
                double currentOutput = motorOutputs[i];
                double absoluteCurrentOutput = Math.abs(currentOutput);
                if (absoluteCurrentOutput > maxOutput) {
                    maxOutput = absoluteCurrentOutput;
                }
            }

            for (int i = 0; i < 4; i++) {
                double currentMotorPower = motorOutputs[i];
                double normalizedMotorPower = currentMotorPower / maxOutput;
                motorOutputs[i] = normalizedMotorPower;
            }


            //slowdown
            for (int i = 0; i < 4; i++) {
                motorOutputs[i] = motorOutputs[i] * Math.min(Math.sqrt(totalDistance / stoppingDistance), 1);
            }
            t.addData("h", h);
            //set the motor powers
            frontLeft.setPower(motorOutputs[0] * power);
            backLeft.setPower(motorOutputs[1] * power);
            frontRight.setPower(motorOutputs[2] * power);
            backRight.setPower(motorOutputs[3] * power);



            pos = odo.getPosition();
            currentX = pos.getX(DistanceUnit.INCH);
            currentY = pos.getY(DistanceUnit.INCH);
            currentHeading = pos.getHeading(AngleUnit.DEGREES);

            t.addData("XCoord", pos.getX(DistanceUnit.INCH));
            t.addData("YCoord",  pos.getY(DistanceUnit.INCH));
            t.addData("Heading", pos.getHeading(AngleUnit.DEGREES));

            t.update();


        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
