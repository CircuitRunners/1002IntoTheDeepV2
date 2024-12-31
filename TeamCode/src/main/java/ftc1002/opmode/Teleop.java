package ftc1002.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime specIntakeTimer = new ElapsedTime();
    int phase = 0;
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
//
//        pin0 = hardwareMap.digitalChannel.get("digital0");
//        pin1 = hardwareMap.digitalChannel.get("digital1");

        specIntakeTimer.reset();
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
            specIntake = (specIntake + 1) % 4; // Advance to the next state (wraps back to 0)
        }
        if (gamepad2.dpad_down && dpadDownReleased) {
            dpadDownPressed = true;
            dpadDownReleased = false;
            specIntake = (specIntake - 1) % 4;
            endEffector.openClaw();
        }




        if (gamepad2.dpad_right && dpadRightReleased) {
            dpadRightPressed = true;
            dpadRightReleased = false;
            specDepo = (specDepo + 1) % 5;
        }


        if (gamepad2.dpad_left && dpadLeftReleased) {
            dpadLeftPressed = true;
            dpadLeftReleased = false;
            specIntake = (specDepo - 1) % 5; // Advance to the next state (wraps back to 0)

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
                specIntakeTimer.reset();
                phase = 0;
                break;

            case 1: // Send lift to 500
                slides.setSlideTarget(400);
                if (slides.liftPos > 380) {
                    endEffector.setPreSubPickupPosition();
                }
                specIntakeTimer.reset();
                phase = 0;
                break;
            case 2: // Combined case for subPickup and deposit
                switch (phase) {
                    case 0: // SubPickup actions
                        if (specIntakeTimer.milliseconds() == 0) {
                            specIntakeTimer.reset(); // Start the timer
                        }
                        endEffector.setSubPickupPosition();
                        endEffector.closeClaw();

                        // Transition to the next phase after 100ms
                        if (specIntakeTimer.milliseconds() >= 150) {
                            phase = 1; // Move to the next phase
                            specIntakeTimer.reset(); // Reset timer for the next phase
                        }
                        break;

                    case 1: // Deposit actions
                        endEffector.setObsDepositPosition();
                        slides.setPivotTarget(12);
                        slides.setSlideTarget(50);

                        // Ready to go back or stay in this state
                        break;
                }
                break;
            case 3: // Additional states if needed
                slides.setPivotTarget(90);
                specIntakeTimer.reset(); // Reset the timer when entering this state
                phase = 0; // Reset phase
                break;
            default:
                specIntakeTimer.reset(); // Reset the timer in the default case
                phase = 0; // Reset phase
                break;
        }

        // State machine for manual control
        switch (specDepo) {
            case 0: // Set pivot to 12, extension to 50, arm idle, claw opens
                slides.setPivotTarget(0);
                slides.setSlideTarget(100);
                endEffector.setWallIntakePosition();
                endEffector.openClaw();
                break;

            case 1: // Send lift to 500
                slides.setPivotTarget(90);
                endEffector.setSpecScore();
                break;
            case 2:
                slides.setSlideTarget(500);
                break;
            case 3:
                slides.setSlideTarget(250);
                if (slides.liftPos < 100) {
                    endEffector.openClaw();
                }
                break;
            case 4:
                slides.setSlideTarget(100);
                break;
            default:
                break;
        }

        slides.update();






        // Debugging and telemetry
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency());
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
