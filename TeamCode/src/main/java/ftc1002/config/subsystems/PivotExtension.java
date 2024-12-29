package ftc1002.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PivotExtension {
    private Telemetry telemetry;
    public DcMotorEx rightLift, leftLift, pivot;
    public AnalogInput pivotEncoder;
    private PIDController liftController, pivotController;
    public static int liftTarget=0, pivotTarget=0;
    public static double liftKP = 0.02, liftKI = 0.0, liftKD = 0.00035, liftKF = 0.15;
    public static double pivotKP = 0.01, pivotKI = 0, pivotKD = 0.0007, pivotKf=0.5;
    public static int liftMax = 1000;
    public static int offset = 24;
    public static int deadband = 15;

    public PivotExtension(HardwareMap hardwareMap, Telemetry telemetry) {
        liftController = new PIDController(liftKP, liftKI, liftKD);
        pivotController = new PIDController(pivotKP, pivotKI, pivotKD);
        this.telemetry = telemetry;

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotTarget = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360) + offset) % 360;
        liftTarget  = Math.round((float) rightLift.getCurrentPosition() / 42) * -1;

    }

    public void update() {
        double liftPID;
        double pivotPID;

        liftController.setPID(liftKP, liftKI, liftKD);
        liftController.setTolerance(deadband);
        pivotController.setPID(pivotKP, pivotKI, pivotKD);
        pivotController.setTolerance(2);

        int liftPos = Math.round((float) rightLift.getCurrentPosition() / 42) * -1;
        int pivotPos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360) + offset) % 360;

        if (Math.abs(liftPos - liftTarget) > deadband) {
            liftPID = square_root(liftController.calculate(liftPos, liftTarget));
        } else {
            liftPID = 0;
        }

        if (Math.abs(pivotPos - pivotTarget) > 2) {
            pivotPID = square_root(pivotController.calculate(pivotPos, pivotTarget));
        } else {
            pivotPID = 0;
        }


        // double pivotPID = square_root(pivotController.calculate(pivotPos, pivotTarget));

        double liftFF = liftKF * Math.sin(Math.toRadians(pivotPos));
        double pivotFF = pivotKf * Math.cos(Math.toRadians(pivotPos)) * ((double) liftPos / liftMax);

        double liftPower = liftPID + liftFF;
        double pivotPower = pivotPID + pivotFF;

        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);
        pivot.setPower(pivotPower);

        telemetry.addData("Lift Pos", liftPos);
        telemetry.addData("Lift Target", liftTarget);
        telemetry.addData("Lift Power", liftPower);
        telemetry.addData("Pivot Pos", pivotPos);
        telemetry.addData("Pivot Target", pivotTarget);
        telemetry.addData("Pivot Power", pivotPower);

        telemetry.update();
    }

    private double square_root(double input) {
        if (input >= 0) {
            return Math.sqrt(input);
        } else {
            return -1 * Math.sqrt(Math.abs(input));
        }
    }

    public boolean isAtTarget(int target) {
        if (Math.abs((Math.round((float) rightLift.getCurrentPosition() / 42) * -1) - target) < deadband) {
            return true;
        } else {
            return false;
        }
    }
}
