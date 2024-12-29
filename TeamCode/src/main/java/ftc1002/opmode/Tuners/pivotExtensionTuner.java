package ftc1002.opmode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc1002.pedroPathing.follower.Follower;


import ftc1002.config.subsystems.PivotExtension;

@TeleOp(name = "PivotExtensionTuner", group = "Tuning")
@Config
public class pivotExtensionTuner  extends OpMode {
    public static double TARGET_LOOP_TIME_MS = 35;
    private ElapsedTime loopTimer = new ElapsedTime();
    private PivotExtension pivotExtension;
    private Follower follower;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivotExtension = new PivotExtension(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);
        telemetry.addLine("PivotLiftTuner Initialized.");
        telemetry.addLine("Press START to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();
        pivotExtension.update();
        follower.update();
        double elapsedTime = loopTimer.milliseconds();
        double delayTime = TARGET_LOOP_TIME_MS - elapsedTime;

        if (delayTime > 0) {
            try {
                Thread.sleep((long) delayTime);
            } catch (InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }

        telemetry.addData("Loop time (ms)", elapsedTime);
        telemetry.addData("Delay Applied (ms)", Math.max(0, delayTime));
        telemetry.update();
    }
}
