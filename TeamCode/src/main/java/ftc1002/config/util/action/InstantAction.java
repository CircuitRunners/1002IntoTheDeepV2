package ftc1002.config.util.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class InstantAction implements Action {
    private final InstantFunction f;

    interface InstantFunction {
        void run();
    }

    public InstantAction(InstantFunction f) {
        this.f = f;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        f.run();
        return false;
    }
}