package ftc1002.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.Locale;

import ftc1002.config.subsystems.MecanumDrive;
import ftc1002.config.subsystems.PivotExtension;
import ftc1002.config.subsystems.EndEffector;

import ftc1002.config.util.GoBildaPinpointDriver;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode{
    MecanumDrive drive;
    PivotExtension slides;
    EndEffector endEffector;

    int pivotState = -1;


    DigitalChannel pin0;
    DigitalChannel pin1;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        drive = new MecanumDrive();
        slides = new PivotExtension(hardwareMap, telemetry);
        endEffector = new EndEffector(hardwareMap);

        endEffector.idle();

        telemetry.addLine("Initializing...");
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

    boolean dpadUpPressed = false;  // Tracks if the dpad_up is pressed
    boolean dpadUpReleased = true; // Ensures dpad_up is released before detecting another press
    int sequenceState = -1;         // Tracks the current state within the sequence

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = driveFieldRelative(forward, right, rotate);

        if (gamepad1.x) {
            pinpoint.resetPosAndIMU(); // Resets the position to 0 and recalibrates the IMU
        }

        if (gamepad1.y) {
            pinpoint.recalibrateIMU(); // Recalibrates the IMU without resetting position
        }

        // Handle manual sequence advancement with dpad_up
        if (gamepad1.dpad_up && dpadUpReleased) {
            dpadUpPressed = true;
            dpadUpReleased = false;
            sequenceState = (sequenceState + 1) % 7; // Advance to the next state (wraps back to 0)
        }

        if (!gamepad1.dpad_up) {
            dpadUpReleased = true;
        }

        if (gamepad1.left_bumper) {
            endEffector.clawOpen();
        }

        if (gamepad1.right_bumper) {
            endEffector.clawClose();
        }

        // State machine for manual control
        switch (sequenceState) {
            case 0: // Set pivot to 12, extension to 50, arm idle, claw opens
                PivotExtension.pivotTarget = 12;
                PivotExtension.liftTarget = 50;
                endEffector.idle();
                endEffector.clawOpen();
                break;

            case 1: // Send lift to 500
                PivotExtension.liftTarget = 500;
                if (slides.isAtTarget(400)) {
                    endEffector.preSubPickup();
                }
                break;
            case 2: // Call subPickup and close claw
                endEffector.subPickup();
                endEffector.clawClose();
                break;
            case 3:
                endEffector.idle();
                PivotExtension.pivotTarget = 12;
                PivotExtension.liftTarget = 50;
                break;
            case 4:
                PivotExtension.liftTarget = 40;
                PivotExtension.pivotTarget = 90;
                break;
            case 5: // Lift to 850
                PivotExtension.liftTarget = 1015;
                endEffector.bucketScore();
                break;
            case 6:
                PivotExtension.liftTarget = 50;
                endEffector.idle();
                break;
            default:
                break;
        }

        slides.update();




        endEffector.update();

        // Debugging and telemetry
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 1", pin1.getState());
        telemetry.addData("Sequence State", sequenceState);
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
