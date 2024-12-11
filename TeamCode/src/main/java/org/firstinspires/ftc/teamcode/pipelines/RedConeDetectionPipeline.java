package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;


    public class RedConeDetectionPipeline extends OpenCvPipeline {
        boolean viewportPaused;
        Mat mask = new Mat();
        Mat output = new Mat();

        Point topLeft = new Point(0, 0);
        Point bottomleft = new Point(159, 119);
        Point topRight = new Point(159, 0);
        Point bottomRight = new Point(319, 119);

        Mat leftTopHalf = new Mat();
        Mat rightTopHalf = new Mat();
        Scalar avgRight;
        Scalar avgLeft;
        Telemetry telemetry;
        int location;
        double redleftVal;
        double redRightVal;

        public Scalar lowerBound = new Scalar(93, 0, 0); //change some boundries based on lighting
        public Scalar upperBound = new Scalar(255, 80, 80);

        public RedConeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {


            //Core.flip(input,input,-1);

            telemetry.addData("rows",input.rows());
            telemetry.addData("Cols", input.cols());
            telemetry.addData("location",location);
            telemetry.update();
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
            Imgproc.blur(input, input, new Size(10,10));
            Core.inRange(input, lowerBound, upperBound, mask);
            output.release();
            Core.bitwise_and(input, input, output, mask);
            leftTopHalf = output.submat(new Rect(topLeft, bottomleft));
            rightTopHalf = output.submat(new Rect(topRight, bottomRight));
            avgLeft = Core.mean(leftTopHalf);
            avgRight = Core.mean(rightTopHalf);
            redleftVal = avgLeft.val[0];
            redRightVal = avgRight.val[0];
            if (redleftVal > 15) { //update thresh hold based on lighting at competition
                location =   0;
                telemetry.addData("Left",0);
                telemetry.update();
                //left
            } else if (redRightVal > 8) { //update thresh hold based on lighting at competition
                location = 1;
                telemetry.addData("Center",1);
                telemetry.update();
                //center
            } else {
                location = 2;
                telemetry.addData("Right",2);
                telemetry.update();
            }

            telemetry.addData("avgLeft", avgLeft);
            telemetry.addData("avgRight", avgRight);
            telemetry.addData("location", location);
            telemetry.update();



            return output;
        }


        public int getLocation() {

            return location;
        }

    }
