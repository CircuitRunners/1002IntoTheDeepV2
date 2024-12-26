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
        pivotExtension = new pivotExtension(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        pivotExtension.update();
    }
}
