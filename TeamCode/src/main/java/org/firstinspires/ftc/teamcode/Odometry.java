package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Odometry implements Runnable {

    private DcMotor vl;
    private DcMotor vr;
    private DcMotor h;
    private double vRadius = 7.5;
    private double xCoord = 0;
    private double yCoord = 0;
    private double heading = 0;
    private double vlStart = 0;
    private double vrStart = 0;
    private double hStart = 0;
    private double encoderTicksPerInch = 1141.94659527;
    private double circumference = 2 * vRadius * Math.PI;
    private double HCircumference = 5.375 * 2 * Math.PI; // Remeasure

    public Odometry(DcMotor vl, DcMotor vr, DcMotor h) {
        this.vl = vl;
        this.vr = vr;
        this.h = h;
        this.vlStart = this.vl.getCurrentPosition();
        this.vrStart = this.vr.getCurrentPosition();
        this.hStart = this.h.getCurrentPosition();
    }

    public void updatePosition() {
        // when the code updates this will allow the code to be correct to the current position
        double vlEnd = vl.getCurrentPosition();
        double vrEnd = vr.getCurrentPosition();
        double hEnd = h.getCurrentPosition();

        // finds the change in position in both wheels/how far each wheel went
        double deltaVL = vlEnd - vlStart;
        double deltaVR = vrEnd - vrStart;

        //averages the 2 numbers so that if they move different amounts you get the right number
        double deltaV = (deltaVL + deltaVR) / 2.0;
        double deltaVIn = deltaV / encoderTicksPerInch;

        //finding the change for both the X and Y
        double deltaXv = (Math.sin(Math.toRadians(heading)) * deltaVIn);
        double deltaYv = (Math.cos(Math.toRadians(heading)) * deltaVIn);

        //Update heading calculations
        double deltaR = (deltaVL - deltaVR) / 2.0;
        double deltaRIn = deltaR/encoderTicksPerInch;
        double deltaRpercent = deltaRIn/circumference;
        double deltaHeading = deltaRpercent * 360;

        //update heading
        heading = heading + deltaHeading;

        double deltaRH = deltaHeading / 360 * HCircumference;

        double deltaHRaw = hEnd - hStart;

        double deltaHInRaw = deltaHRaw / encoderTicksPerInch;

        double deltaHIn = deltaHInRaw + deltaRH;

        double deltaXh = (Math.sin(Math.toRadians(heading + 90))) * deltaHIn;
        double deltaYh = (Math.cos(Math.toRadians(heading + 90))) * deltaHIn;

        //Update x and y pos
        xCoord = xCoord + deltaXh + deltaXv;
        yCoord = yCoord + deltaYh + deltaYv;

        vlStart = vlEnd;
        vrStart = vrEnd;
        hStart = hEnd;
    }

    public double getXCoordinate() {
        return xCoord;
    }

    public double getYCoordinate() {
        return yCoord;
    }

    public double getHeading() {
        return heading;
    }

    public void run() {
        while(true) {
            try {
                this.updatePosition();
            } catch (Exception e) {
                throw e;
            }
        }
    }

}
