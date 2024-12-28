package ftc1002.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.Locale;

import ftc1002.config.subsystems.MecanumDrive;

import ftc1002.config.util.GoBildaPinpointDriver;
import ftc1002.pedroPathing.localization.Pose;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode{
    MecanumDrive drive = new MecanumDrive();
    DigitalChannel pin0;
    DigitalChannel pin1;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        drive.init(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        configurePinpoint();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();
        telemetry.addLine("Ready!");
        telemetry.update();

        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        Pose2D pos = driveFieldRelative(forward, right, rotate);
        if (gamepad1.x){
            pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }
        if (gamepad1.y){
            pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
        }
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 1", pin1.getState());
        telemetry.update();

    }

    public void configurePinpoint() {
        pinpoint.setOffsets(153.22873, -68.86620); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    private Pose2D driveFieldRelative(double forward, double right, double rotate) {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();
        double robotAngle = Math.toRadians(pos.getHeading(AngleUnit.DEGREES));
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
        return pos;
    }
}
