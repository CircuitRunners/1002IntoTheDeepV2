// Archive

package org.firstinspires.ftc.teamcode.opmode.Archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystems.Folder.pivotLiftArchive;

/**
 * A simple TeleOp for tuning the pivotLift PID controllers.
 *
 *  1. Start in LIFT_TUNING mode.
 *  2. Use the DPAD (or the dashboard) to change Kp, Ki, Kd, etc.
 *  3. Press the X button (for example) to toggle to PIVOT_TUNING mode.
 *  4. Repeat for pivot.
 *
 * Adjust to your liking!
 */
@Config
@TeleOp(name="PivotLiftTunerTEAST", group="Tuning")
public class pivotLiftTunerArchive extends LinearOpMode {

    // -- LIFT PID GAINS (tunable from Dashboard) --
    public static double liftKp = 0;
    public static double liftKi = 0.0;
    public static double liftKd = 0;
    public static double liftKf = 0;  // used as feedforward for the lift (sin-based)
    public static int liftTarget = 0;

    // -- PIVOT PID GAINS --
    public static double pivotKp = 0;
    public static double pivotKi = 0;
    public static double pivotKd = 0;
    public static int pivotTarget = 0;

    // We'll keep a reference to our pivotLift subsystem
    private pivotLiftArchive pLift;

    // We'll make an enum to switch between states.
    public enum TuningState {
        LIFT_TUNING,
        PIVOT_TUNING
    }

    private TuningState currentState = TuningState.LIFT_TUNING;

    @Override
    public void runOpMode() {
        // Instantiate pivotLift subsystem
        pLift = new pivotLiftArchive(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the subsystem
        pLift.init();

        // Set initial PID values in case the static fields differ from the default constructor
        pLift.liftKP = liftKp;
        pLift.liftKI = liftKi;
        pLift.liftKD = liftKd;
        pLift.liftKF = liftKf;
        pLift.liftTarget = liftTarget;

        pLift.pivotKP = pivotKp;
        pLift.pivotKI = pivotKi;
        pLift.pivotKD = pivotKd;
        pLift.pivotTarget = pivotTarget;

        telemetry.addLine("PivotLiftTuner Initialized.");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        // -----------
        // MAIN LOOP
        // -----------
        while (opModeIsActive()) {

            // Read gamepad input to toggle between Lift Tuning and Pivot Tuning
            if (gamepad1.x) {
                // simple toggle: if we press 'X', we change to the other mode
                if (currentState == TuningState.LIFT_TUNING) {
                    currentState = TuningState.PIVOT_TUNING;
                } else {
                    currentState = TuningState.LIFT_TUNING;
                }
                // just to prevent double toggles from a single press,
                // wait a bit for release
                sleep(300);
            }

            // If we wanted to adjust PID values on the fly via the gamepad, we could do:
            // if (gamepad1.dpad_up) liftKp += 0.001;
            // if (gamepad1.dpad_down) liftKp -= 0.001;
            // ...
            // For now, let's assume we do it from the dashboard.

            // Update pivotLift’s internal PID variables to match dashboard values in real-time
            pLift.liftKP = liftKp;
            pLift.liftKI = liftKi;
            pLift.liftKD = liftKd;
            pLift.liftKF = liftKf;
            pLift.liftTarget = liftTarget;

            pLift.pivotKP = pivotKp;
            pLift.pivotKI = pivotKi;
            pLift.pivotKD = pivotKd;
            pLift.pivotTarget = pivotTarget;

            pLift.updateLiftPIDF();  // runs the closed-loop control for the lift
            pLift.updatePivotPIDF();



            // Telemetry
            telemetry.addLine("=== PivotLiftTuner ===");
            telemetry.addData("Lift Pos", pLift.getLiftPos());
            telemetry.addData("Lift Target", pLift.getLiftTarget());
            telemetry.addData("Lift Motor Power", pLift.rightLift.getPower());
            telemetry.addData("Pivot Angle", pLift.getPivotAngle());
            telemetry.addData("Pivot Target", pLift.getPivotTarget());
            telemetry.addData("Pivot Motor Power", pLift.pivot.getPower());
            telemetry.addData("Lift Kp", liftKp);
            telemetry.addData("Lift Ki", liftKi);
            telemetry.addData("Lift Kd", liftKd);
            telemetry.addData("Lift Kf", liftKf);
            telemetry.addData("Pivot Kp", pivotKp);
            telemetry.addData("Pivot Ki", pivotKi);
            telemetry.addData("Pivot Kd", pivotKd);
            telemetry.addData("Pivot Kf", pLift.pivotKf);
            telemetry.addLine("Press X to toggle Tuning State.");
            telemetry.update();
        }
    }
}