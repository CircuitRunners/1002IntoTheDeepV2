package ftc1002.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class EndEffector {
    private final Servo armServoL, armServoR, pivotServo, wristServo, clawServo;
    // Define a position double variable for each servo and make it public static, don't assign a value and do all the variable declarations on one line
    public static double armPosition, pivotPosition, wristPosition, clawPosition;



    public EndEffector(HardwareMap hardwareMap) {
        armServoL = hardwareMap.get(Servo.class, "armServoL");
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

    public void idle() {
        setPositions(0.8, 0.5, 0.5);
    }
    public void bucketScore() {
        setPositions(0.8, 0.4, 0.5);
    }

    public void preSubPickup() {
        setPositions(0.8, 1, 0.5);
    }

    public void subPickup() {
        setPositions(0.86, 0.98, 0.5);
    }

    public void obsDeposit() {
        setPositions(0.2, 0.7, 0.5);
    }

    public void clawOpen() {
        clawPosition = 0.7;
    }

    public void clawClose() {
        clawPosition = 0.3;
    }

    // write an update method that sets all the positions for the servo
    public void update() {
        armServoL.setPosition(armPosition);
        armServoR.setPosition(armPosition);
        setPivotPosition(pivotPosition);
        setWristPosition(wristPosition);
        setClawPosition(clawPosition);
    }

    public void setPositions(double armPos, double clawPos, double pivotPos, double wristPos) {
        armPosition = armPos;
        wristPosition = wristPos;
        pivotPosition = pivotPos;
        clawPosition = clawPos;
    }

    public void setPositions(double armPos, double pivotPos, double wristPos) {
        armPosition = armPos;
        wristPosition = wristPos;
        pivotPosition = pivotPos;
    }
}
