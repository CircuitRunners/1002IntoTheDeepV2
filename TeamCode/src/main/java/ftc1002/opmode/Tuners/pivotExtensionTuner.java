package ftc1002.opmode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftc1002.config.subsystems.pivotExtension;

@TeleOp(name = "PivotExtensionTuner", group = "Tuning")
public class pivotExtensionTuner  extends OpMode {
    private pivotExtension pivotExtension;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivotExtension = new pivotExtension(hardwareMap, telemetry);
        telemetry.addLine("PivotLiftTuner Initialized.");
        telemetry.addLine("Press START to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pivotExtension.update();
    }
}
