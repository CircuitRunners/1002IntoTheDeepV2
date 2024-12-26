package ftc1002.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class pivotExtension {
    private Telemetry telemetry;
    public DcMotorEx rightLift, leftLift, pivot;
    public AnalogInput pivotEncoder;
    private PIDController liftController, pivotController;
    public static int liftTarget=0, pivotTarget=0;
    public static double liftKP = 0, liftKI = 0.0, liftKD = 0, liftKF = 0;
    public static double pivotKP = 0, pivotKI = 0, pivotKD = 0, pivotKf=0;
    public static int liftMax = 0;

    public pivotExtension(HardwareMap hardwareMap, Telemetry telemetry) {
        liftController = new PIDController(liftKP, liftKI, liftKD);
        pivotController = new PIDController(pivotKP, pivotKI, pivotKD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void update() {
        liftController.setPID(liftKP, liftKI, liftKD);
        pivotController.setPID(pivotKP, pivotKI, pivotKD);

        int liftPos = Math.round((float) rightLift.getCurrentPosition() / 42);
        int pivotPos = (int) Math.round(pivotEncoder.getVoltage() / 3.2 * 360) - 185;

        double liftPID = Math.sqrt(liftController.calculate(liftPos, liftTarget));
        double pivotPID = Math.sqrt(pivotController.calculate(pivotPos, pivotTarget));

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
}
