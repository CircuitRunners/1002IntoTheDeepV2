package ftc1002.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class EndEffector {

    // Public static variables for FTC Dashboard
    public static double armPosition = 0.0;
    public static double pivotPosition = 0.0;
    public static double wristPosition = 0.0;
    public static double clawPosition = 0.0;

    // Private Servo instances
    private final Servo armServoLeft;
    private final Servo armServoRight;
    private final Servo pivotServo;
    private final Servo wristServo;
    private final Servo clawServo;

    // Constructor
    public EndEffector(HardwareMap hardwareMap) {
        armServoLeft = hardwareMap.get(Servo.class, "armServoL");
        armServoRight = hardwareMap.get(Servo.class, "armServoR");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    // Public setters
    public void setArmPosition(double position) {
        armServoLeft.setPosition(position);
        armServoRight.setPosition(position);
        armPosition = position;
    }

    public void setPivotPosition(double position) {
        pivotServo.setPosition(position);
        pivotPosition = position;
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
        wristPosition = position;
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
        clawPosition = position;
    }

    // Public getters
    public double getArmPosition() {
        return armServoLeft.getPosition();
    }

    public double getPivotPosition() {
        return pivotServo.getPosition();
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    // Utility methods for preset positions
    public void setIdlePosition() {
        setPositions(0.8, 0.4, 0.5, clawPosition);
    }

    public void setBucketScorePosition() {
        setPositions(0.8, 0.2, pivotPosition, clawPosition);
    }

    public void setPreSubPickupPosition() {
        setPositions(0.78, 0.9, wristPosition, clawPosition);
    }

    public void setSubPickupPosition() {
        setPositions(0.82, 0.9, wristPosition, clawPosition);
    }

    public void setObsDepositPosition() {
        setPositions(0.2, 0.5, 0.5, clawPosition);
    }

    public void setWallIntakePosition() {
        setPositions(1, 0.3, 0.5, 0.7);
    }

    public void setSpecScore() {
        setPositions(0.3, 0.4, 0.5, 0.3);
    }

    public void openClaw() {
        setClawPosition(0.7);
    }

    public void closeClaw() {
        setClawPosition(0.3);
    }

    // Incremental adjustments
    public void incrementWristPosition(double step) {
        setWristPosition(Range.clip(wristPosition + step, 0.0, 1.0));
    }

    public void decrementWristPosition(double step) {
        setWristPosition(Range.clip(wristPosition - step, 0.0, 1.0));
    }

    public void incrementPivotPosition(double step) {
        setPivotPosition(Range.clip(pivotPosition + step, 0.0, 1.0));
    }

    public void decrementPivotPosition(double step) {
        setPivotPosition(Range.clip(pivotPosition - step, 0.0, 1.0));
    }

    // Update method to apply positions
    public void init() {
        setArmPosition(armPosition);
        setPivotPosition(pivotPosition);
        setWristPosition(wristPosition);
        setClawPosition(clawPosition);
    }

    // Unified setters for multiple positions
    public void setPositions(double armPos, double pivotPos, double wristPos, double clawPos) {
        setArmPosition(armPos);
        setPivotPosition(pivotPos);
        setWristPosition(wristPos);
        setClawPosition(clawPos);
    }

    public void setPositions(double armPos, double pivotPos, double wristPos) {
        setArmPosition(armPos);
        setPivotPosition(pivotPos);
        setWristPosition(wristPos);
    }
}