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

    /*public static void goToPosition(double x, double y, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, Odometry.java odometry) {
        goToX(x, frontLeft, backLeft, frontRight, backRight, odometry);
        goToY(y, frontLeft, backLeft, frontRight, backRight, odometry);
    }*/
   /* public static void goToPosition(double x, double y, Odometry odometry, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        double currentX = odometry.getXCoordinate();
        double currentY = odometry.getYCoordinate();
        double currentHeading = odometry.getHeading();
        double changeX = x - currentX;
        double changeY = y - currentY;
        double angle = Math.atan2(changeY, changeX);
        rotate(angle, odometry, leftFront, leftBack, rightFront, rightBack);
        goToY(y, leftFront, leftBack, rightFront, rightBack, odometry);
    }

    public static void rotate(double goalAngle, Odometry odometry, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        double currentHeading = odometry.getHeading();
        while (goalAngle != currentHeading) {
            currentHeading = odometry.getHeading(); // Get heading and check how close we are
                leftFront.setPower(1);
                leftBack.setPower(1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);  // Set powers to rotate
        }
    }*/
    public static double DIAMETER = 15;

    public static double[] goToX(double x, Odometry odometry) {
        /**
         * returns a list of doubles in the following order:
         * frontLEFT
         * BACKLEFT
         * FRONTRIGHT
         * BACKRIGHT
         * Distance
         */




        double p = 15; //Stopping distance
        double currentX = odometry.getXCoordinate(); // where we are now
        double distance = x - currentX; // how far the robot is form the input
        double power = distance / p;

            if (power > 1)
                power = 1;
            else if (power < -1)
                power = -1;

            power = power/2.0;

            return new double[]{power, -power, -power, power, distance};

    }

    public static void  goToY(double y, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, Odometry odometry) {
        double p = 15; //Stopping distance
        double currentY = odometry.getYCoordinate(); // where we are now
        double distance = y - currentY; // how far the robot is form the input
        double power = distance / p;

        while (distance > 0.25) {

            currentY = odometry.getYCoordinate(); // where we are now
            distance = y - currentY; // how far the robot is form the input
            power = distance / p;

            if (power > 1)
                power = 1;
            else if (power < -1)
                power = -1;

            power = power/2.0;

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    }



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
            //get starting position
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
