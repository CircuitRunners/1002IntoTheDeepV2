package ftc1002.config.subsystems;

import static ftc1002.config.util.RobotConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import ftc1002.config.util.RobotConstants;
import ftc1002.config.util.action.RunAction;

public class pivotLift {
    private Telemetry telemetry;
    public DcMotorEx rightLift, leftLift, pivot;
    public AnalogInput pivotEncoder;
    public boolean manualLift = false;
    public boolean manualPivot = false;
    public boolean hang =  false;
    public int pos, bottom;
    public int pivotStart = 10;
    public PIDController liftPID, pivotPID;
    public static int liftTarget, pivotTarget;
    public static double liftKP = 0.02, liftKI = 0.0, liftKD = 0.001, liftKF = 0.0;
    public static double pivotKP = 0.02, pivotKI = 0.0, pivotKD = 0.001;

    public pivotLift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPID = new PIDController(liftKP, liftKI, liftKD);
        pivotPID = new PIDController(pivotKP, pivotKI, pivotKD);
    }

    public void updateLiftPIDF() {
        if (!manualLift) {
            liftPID.setPID(liftKP, liftKI, liftKD);

            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double pid = liftPID.calculate(getLiftPos(), liftTarget);
            double ff = liftKF * Math.sin(Math.toRadians(getPivotAngle()));
            double power = pid + ff;

            rightLift.setPower(power);
            leftLift.setPower(power);

            telemetry.addData("lift pos", getLiftPos());
            telemetry.addData("lift target", liftTarget);
        }
    }

    public void updatePivotPIDF() {
        if (!manualPivot) {
            double ff;
            pivotPID.setPID(pivotKP, pivotKI, pivotKD);
            double pid = pivotPID.calculate(getPivotAngle(), pivotTarget);
            if (getPivotAngle() > 90) {
                ff = linearlyScaledFF() * Math.cos(Math.toRadians(getPivotAngle()-90));
            } else {
                ff = linearlyScaledFF() * Math.cos(Math.toRadians(90-getPivotAngle()));
            }
            double power = pid + ff;

            pivot.setPower(power);

            telemetry.addData("pivot angle", getPivotAngle());
            telemetry.addData("pivot target", pivotTarget);
        }
    }

    public void manualLift(double n) {
        manualLift = true;

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(hang) {
            n = -0.75;
        }

        rightLift.setPower(n);
        leftLift.setPower(n);
        liftTarget = getLiftPos();

        if (rightLift.getCurrent(CurrentUnit.AMPS) > 5 && !hang) {
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftTarget = 0;
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void manualPivot(double n) {
        manualPivot = true;
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setPower(n);
    }

    // Util
    public void liftTargetCurrent() {
        liftTarget = getLiftPos();
    }

    public double getLiftTarget() {
        return liftTarget;
    }

    public double getPivotTarget() {
        return pivotTarget;
    }

    public void setLiftTarget(int target) {
        liftTarget = target;
    }

    public void setPivotTarget(int target) {
        pivotTarget = target;
    }

    public void addToLiftTarget(int target) {
        liftTarget += target;
    }

    public void addToPivotTarget(int target) {
        pivotTarget += target;
    }

    public int getLiftPos() {
        pos = rightLift.getCurrentPosition() - bottom;
        return pos;
    }

    public int getPivotAngle() {
        // round to the nearest degree
        return (int) Math.round(pivotEncoder.getVoltage() / 3.2 * 360);
    }

    public double linearlyScaledFF() {
        double min_extension = 0.0;
        double max_extension = 100.0;
        double min_feedforward = 0.05;
        double max_feedforward = 0.1;
        return (max_feedforward - min_feedforward) / (max_extension - min_extension) * (getLiftPos() - min_extension) + min_feedforward;
    }
    // OpMode
    public void init() {
        liftPID.setPID(liftKP, liftKI, liftKD);
        bottom = getLiftPos();
        pivotPID.setPID(pivotKP, pivotKI, pivotKD);
    }

    public void start() {
        liftTarget = bottom;
        pivotTarget = pivotStart;
    }


    // Presets


}
