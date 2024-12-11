/**package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

// Shared State for Position
class RobotState {
    private static Pose2D position = new Pose2D();

    public static synchronized void setPosition(Pose2D pos) {
        position = pos;
    }

    public static synchronized Pose2D getPosition() {
        return position;
    }
}

// Listener Interface for Position Updates
interface PositionListener {
    void onPositionUpdated(Pose2D position);
}

// Example Position Handler
class PositionHandler implements PositionListener {
    @Override
    public void onPositionUpdated(Pose2D position) {
        double x = position.getX(DistanceUnit.MM);
        double y = position.getY(DistanceUnit.MM);
        double heading = position.getHeading(AngleUnit.DEGREES);

        System.out.println("Updated Position: X=" + x + ", Y=" + y + ", Heading=" + heading);
    }
}

@TeleOp(name = "goBILDA® PinPoint Odometry Example", group = "Linear OpMode")
public class ChatCodeTransferData extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Position Handler Instance
        PositionHandler handler = new PositionHandler();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            // Get and Store Position
            Pose2D pos = odo.getPosition();
            RobotState.setPosition(pos);

            // Notify Listener
            handler.onPositionUpdated(pos);

            // Telemetry for Debugging
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    vel.getX(DistanceUnit.MM),
                    vel.getY(DistanceUnit.MM),
                    vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
            telemetry.update();

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
            telemetry.addData("REV Hub Frequency: ", frequency);
        }
    }
}
 */
