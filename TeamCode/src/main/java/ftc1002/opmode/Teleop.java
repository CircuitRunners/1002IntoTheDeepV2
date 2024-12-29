package ftc1002.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.Locale;

import ftc1002.config.subsystems.EndEffector;
import ftc1002.config.subsystems.MecanumDrive;

import ftc1002.config.util.GoBildaPinpointDriver;
import ftc1002.config.util.action.Action;
import ftc1002.config.util.action.SequentialAction;
import ftc1002.config.util.action.SleepAction;


@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    DigitalChannel pin0;
    DigitalChannel pin1;
    GoBildaPinpointDriver pinpoint;

    private EndEffector endEffector;

//    private enum PivotState {INTAKESUB, SPECIMEN, BASKET}
//
//    private PivotState pivotState = PivotState.BASKET;


    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        drive.init(hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
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
        if (gamepad1.x) {
            pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }
        if (gamepad1.y) {
            pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

        endEffectorIncrimentControls();


        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Status", pinpoint.getDeviceStatus());

        telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 1", pin1.getState());
        telemetry.addData("wrist pos", "%.2f", endEffector.getWristPosition());
        telemetry.addData("pivot pos", "%.2f", endEffector.getPivotPosition());
        telemetry.addData("arm pos", "%.2f", endEffector.getArmPosition());
        telemetry.addData("claw pos", "%.2f", endEffector.getClawPosition());
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


//    public void endEffectorControl() {
//        switch (pivotState) {
//            case BASKET:
//                //pivot.setPosition(basket)
//                // slides.setPosition(transfer)
//                endEffector.transfer();
//
//                if (gamepad2.dpad_up && endEffector.getArmPosition() == 0) {
//                    endEffector.scoreBucket();
//                    //slides.scoreBucket
//                }
//               else if (gamepad2.dpad_up ) {
//                    // slides.setPosition(transfer)
//                    endEffector.transfer();
//                }
//
//                if (gamepad2.dpad_down  /*&&slides.getPosition == slides.scoreBucket*/) {
//                    endEffector.openClaw();
//                    //slides.transfer
//                    endEffector.transfer();
//                } else if (gamepad2.dpad_down/*slides.getPosition == slides.transfer*/) {
//                    pivotState = PivotState.INTAKESUB;
//                }
//                if (gamepad2.dpad_left || gamepad2.dpad_right) {
//                    pivotState = PivotState.SPECIMEN;
//                }
//                if (gamepad2.y) {
//                    //slides.wallIntake
//                    endEffector.intakeWall();
//                }
//                else if (gamepad2.a) {
//                    //slides.transfer
//                    endEffector.intakeGround();
//                }
//                break;
//            case SPECIMEN:
//                //pivot.setPosition(specimen)
//                //slides.setPosition(SpecimenRetracted)
//                endEffector.scoreSpecimen();
//                if (gamepad2.dpad_up) {
//                    //slides.setPosition(SpecimenExtended)
//                }
//                if (gamepad2.dpad_down /* &&slides.getPosition == slides.SpecimenExtended) */) {
//                    //slides.setPosition(scoreSpecimen)
//                } else if (gamepad2.dpad_down /* && slides.getPosition == slides.Position(specimenScore)*/) {
//                    endEffector.openClaw();
//                    //slides.specimenRetracted
//                    pivotState = PivotState.BASKET;
//
//                }
//                break;
//            case INTAKESUB:
//                //pivot.setPosition(intakeSub)
//                if (gamepad2.dpad_down/*&&slides.getPosition == slides.setPosition(intakeSubRetracted)*/) {
//                    // slides.setPosition(intakeSubExtended);
//                    endEffector.preSubIntake();
//                } else if (gamepad2.dpad_down /*&&slides.getPosition == slides.setPositon(intakeSubExtended) */) {
//                    intakeSubPickup();
//                }
//                if (gamepad2.dpad_left /*&&slides.getPosition == slides.setPositon(intakeSubExtended) */) {
//                    endEffector.preSubIntake();
//                } else if (gamepad2.dpad_right /*&&slides.getPosition == slides.setPositon(intakeSubExtended) */) {
//                    endEffector.preSubIntake();
//                }
//                if (gamepad2.dpad_up) {
//                    endEffector.postIntakeSub();
//                    //slides.intakeSubRetract
//                    pivotState = PivotState.BASKET;
//                }
//                break;
//
//        }
//    }


    public void endEffectorIncrimentControls() {
        if (gamepad2.right_stick_y > 0.5) {
            endEffector.armLincrement();
            endEffector.armRincrement();
        }
        if (gamepad2.right_stick_y < -0.5) {
            endEffector.armLdecrement();
            endEffector.armRdeincrement();
        }
        if (gamepad2.right_stick_x > 0.5) {
            endEffector.wristincrement();

        }
        if (gamepad2.right_stick_x < -0.5) {
            endEffector.wristdecrement();
        }

        if (gamepad2.left_trigger > 0.2) {
            endEffector.pivotincrement();
        }
        if (gamepad2.right_trigger > 0.2) {
            endEffector.pivotdeincrement();
        }
        if (gamepad2.left_bumper) {
            endEffector.clawincrement();
        }
        if (gamepad2.right_bumper) {
            endEffector.clawidencrement();
        }
    }

    public Action intakeSubPickup() {
        return new SequentialAction(
                endEffector.intakeSubTeleOp,
                new SleepAction(0.2),
                endEffector.closeClaw,
                new SleepAction(0.2),
                endEffector.postIntakeSub
        );
    }
}




