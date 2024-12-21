package ftc1002.config.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class endEffector {
    private final Servo armServoL, armServoR, pivotServo, wristServo, clawServo;

    public endEffector(HardwareMap hardwareMap) {
        armServoL = hardwareMap.get(Servo.class, "armServoL");
        armServoL.setDirection(Servo.Direction.REVERSE);
        armServoR = hardwareMap.get(Servo.class, "armServoR");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    // Setters
    public void setArmPosition(double position) {
        armServoL.setPosition(position);
        armServoR.setPosition(position);
    }
    public void setArmServoLPosition(double position) { armServoL.setPosition(position); }
    public void setArmServoRPosition(double position) { armServoR.setPosition(position); }
    public void setPivotPosition(double position) { pivotServo.setPosition(position); }
    public void setWristPosition(double position) { wristServo.setPosition(position); }
    public void setClawPosition(double position) { clawServo.setPosition(position); }

    // Getters
    public double getArmPosition() { return armServoL.getPosition(); }
    public double getPivotPosition() { return pivotServo.getPosition(); }
    public double getWristPosition() { return wristServo.getPosition(); }
    public double getClawPosition() { return clawServo.getPosition(); }
}
