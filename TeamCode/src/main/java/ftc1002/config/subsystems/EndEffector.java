package ftc1002.config.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc1002.config.util.action.RunAction;

public class EndEffector {
    private final Servo armServoL;
    private final Servo armServoR;
    private final Servo pivotServo;
    private final Servo wristServo;
    private final Servo clawServo;

    public RunAction preSubIntake, intakeSubTeleOp, intakeSubAuto,postIntakeSub, intakeGround,intakeWall, scoreBucket, scoreSpecimen, transfer, openClaw, closeClaw;

    public EndEffector(HardwareMap hardwareMap) {
        armServoL = hardwareMap.get(Servo.class, "armServoL");
        armServoL.setDirection(Servo.Direction.REVERSE);
        armServoR = hardwareMap.get(Servo.class, "armServoR");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        preSubIntake = new RunAction(this::preSubIntake);
        intakeSubTeleOp = new RunAction(this::intakeSubTeleOp);
        intakeSubAuto = new RunAction(this::intakeSubAuto);
        postIntakeSub = new RunAction(this::postIntakeSub);
        intakeGround = new RunAction(this::intakeGround);
        intakeWall = new RunAction(this::intakeWall);
        scoreBucket = new RunAction(this::scoreBucket);
        scoreSpecimen = new RunAction(this::scoreSpecimen);
        transfer = new RunAction(this::transfer);
        openClaw = new RunAction(this::openClaw);
        closeClaw = new RunAction(this::closeClaw);

    }


    // Setters
    public void setArmPosition(double position) {
        armServoL.setPosition(position);
        armServoR.setPosition(position);
    }

    public void setArmServoLPosition(double position) {
        armServoL.setPosition(position);
    }

    public void setArmServoRPosition(double position) {
        armServoR.setPosition(position);
    }

    public void setPivotPosition(double position) {
        pivotServo.setPosition(position);
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    // Getters
    public double getArmPosition() {
        return armServoL.getPosition();
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

    public void preSubIntake() {
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(0);
        setClawPosition(0.7);
    }

    public void intakeSubTeleOp() {
        setArmPosition(0);
        setPivotPosition(0);
        setClawPosition(.7);

    }

    public void intakeSubAuto() {
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(.7);
        setClawPosition(.7);

    }

    public void postIntakeSub() {
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(.5);
        setClawPosition(.22);
    }

    public void intakeWall(){
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(0);
    }

    public void intakeGround(){
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(0);

    }

    public void scoreBucket(){
        setArmPosition(0.3);
        setPivotPosition(0.74);
        setWristPosition(0.5);
      //  setClawPosition(0.22);
    }
    public void scoreSpecimen(){
        setArmPosition(0);
        setPivotPosition(0);
        setWristPosition(0);
      //  setClawPosition(0);
    }
    public void transfer(){
        setArmPosition(0);
        setPivotPosition(1);
        setWristPosition(0.5);
        setClawPosition(0.22);
    }

    public void openClaw(){
        setClawPosition(.7);
    }

    public void closeClaw(){
        setClawPosition(.22);
    }

    public void wristincrement() {wristServo.setPosition(wristServo.getPosition() + 0.02);}
    public void armLincrement() {armServoL.setPosition(armServoL.getPosition() + 0.02);}
    public void armRincrement() {armServoR.setPosition(armServoR.getPosition() + 0.02);}
    public void pivotincrement() {pivotServo.setPosition(pivotServo.getPosition() + 0.02);}
    public void clawincrement() {clawServo.setPosition(clawServo.getPosition() + 0.02);}
    public void wristdecrement() {wristServo.setPosition(wristServo.getPosition() - 0.02);}
    public void armLdecrement() {armServoL.setPosition(armServoL.getPosition() - 0.02);}
    public void armRdeincrement() {armServoR.setPosition(armServoR.getPosition() - 0.02);}
    public void pivotdeincrement() {pivotServo.setPosition(pivotServo.getPosition() - 0.02);}
    public void clawidencrement() {clawServo.setPosition(clawServo.getPosition() - 0.02);}

}
