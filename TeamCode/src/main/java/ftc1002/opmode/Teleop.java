package ftc1002.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.List;
import java.util.Locale;

import ftc1002.config.subsystems.MecanumDrive;
import ftc1002.config.subsystems.Deposit;
import ftc1002.config.subsystems.EndEffector;

import ftc1002.config.util.GoBildaPinpointDriver;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode{
    MecanumDrive drive;
    Deposit slides;
    EndEffector endEffector;


    DigitalChannel pin0;
    DigitalChannel pin1;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new MecanumDrive();
        slides = new Deposit(hardwareMap, telemetry, false);
        endEffector = new EndEffector(hardwareMap);

        endEffector.setIdlePosition();

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
    boolean dpadUpReleased = true; // Ensures dpad_up is released before detecting another
    boolean dpadDownPressed = false;  // Tracks if the dpad_up is pressed
    boolean dpadDownReleased = true; // Ensures dpad_up is released before detecting another // press

    boolean dpadRightPressed = false;  // Tracks if the dpad_up is pressed
    boolean dpadRightReleased = true; // Ensures dpad_up is released before detecting another
    boolean dpadLeftPressed = false;  // Tracks if the dpad_up is pressed
    boolean dpadLeftReleased = true; // Ensures dpad_up is released before detecting another // press
    int specIntake = -1;         // Tracks the current state within the sequence
    int specDepo = -1;

    @Override
    public void loop() {
        if ((specIntake != -1) && (specDepo != -1)){
            specIntake = -1;
            specDepo = -1;
        }
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
        if (gamepad2.dpad_up && dpadUpReleased) {
            dpadUpPressed = true;
            dpadUpReleased = false;
            specIntake = (specIntake + 1) % 5; // Advance to the next state (wraps back to 0)
        }
        if (gamepad2.dpad_down && dpadDownReleased) {
            dpadDownPressed = true;
            dpadDownReleased = false;
            specIntake = (specIntake - 1) % 5;
            endEffector.openClaw();
        }




        if (gamepad2.dpad_right && dpadRightReleased) {
            dpadRightPressed = true;
            dpadRightReleased = false;
            specDepo = (specDepo + 1) % 3;
        }


        if (gamepad2.dpad_left && dpadLeftReleased) {
            dpadLeftPressed = true;
            dpadLeftReleased = false;
            specIntake = (specDepo - 1) % 3; // Advance to the next state (wraps back to 0)

        }



        if (!gamepad2.dpad_up) {
            dpadUpReleased = true;
        }

        if (!gamepad2.dpad_down) {
            dpadDownReleased = true;
        }

        if (!gamepad2.dpad_left) {
            dpadLeftReleased = true;
        }

        if (!gamepad2.dpad_right) {
            dpadRightReleased = true;
        }

        if (gamepad2.left_bumper) {
            endEffector.openClaw();
        }

        if (gamepad2.right_bumper) {
            endEffector.closeClaw();
        }

        if (slides.pivotTarget > 45) {
            if (gamepad2.left_trigger > 0.5) {
                endEffector.decrementPivotPosition(0.02);
            } else if (gamepad2.right_trigger > 0.5) {
                endEffector.incrementPivotPosition(0.02);
            }
        } else {
            if (gamepad2.left_trigger > 0.5) {
                endEffector.decrementWristPosition(0.02);
            } else if (gamepad2.right_trigger > 0.5) {
                endEffector.incrementWristPosition(0.02);
            }
        }




        // State machine for manual control
        switch (specIntake) {
            case 0: // Set pivot to 12, extension to 50, arm idle, claw opens
                slides.setPivotTarget(0);
                slides.setSlideTarget(50);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                break;

            case 1: // Send lift to 500
                slides.setSlideTarget(400);
                if (slides.liftPos > 380) {
                    endEffector.setPreSubPickupPosition();
                }
                break;
            case 2: // Call subPickup and close claw
                endEffector.setSubPickupPosition();
                endEffector.closeClaw();
                break;
            case 3:
                endEffector.setObsDepositPosition();
                slides.setPivotTarget(12);
                slides.setSlideTarget(50);
                break;
            case 4:
                slides.setPivotTarget(90);
            default:
                break;
        }

        // State machine for manual control
        switch (specDepo) {
            case 0: // Set pivot to 12, extension to 50, arm idle, claw opens
                slides.setPivotTarget(28);
                slides.setSlideTarget(200);
                endEffector.setIdlePosition();
                endEffector.openClaw();
                break;

            case 1: // Send lift to 500
                slides.setPivotTarget(90);
                slides.setSlideTarget(375);
                endEffector.setSpecScore();
                break;
            case 2:
                slides.setSlideTarget(850);
            default:
                break;
        }

        slides.update();






        // Debugging and telemetry
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 1", pin1.getState());
        telemetry.addData("Sequence State", specIntake);
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
