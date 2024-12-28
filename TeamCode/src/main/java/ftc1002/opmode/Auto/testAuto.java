package ftc1002.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import ftc1002.pedroPathing.follower.Follower;
import ftc1002.pedroPathing.localization.Pose;
import ftc1002.pedroPathing.pathGeneration.BezierCurve;
import ftc1002.pedroPathing.pathGeneration.BezierLine;
import ftc1002.pedroPathing.pathGeneration.PathChain;
import ftc1002.pedroPathing.pathGeneration.Point;
import ftc1002.pedroPathing.util.Timer;

import ftc1002.config.subsystems.pivotExtension;

@Autonomous(name = "testAuto")
public class testAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private pivotExtension slides;

    private Pose startPos = new Pose(0,0, Math.toRadians(0));
    private Pose interPos = new Pose(24, -24, Math.toRadians(90));
    private Pose endPos = new Pose(24, 24, Math.toRadians(45));

    private PathChain path;

    public void buildPaths() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPos), new Point(interPos)))
                .setLinearHeadingInterpolation(startPos.getHeading(), interPos.getHeading())
                .addPath(new BezierLine(new Point(interPos), new Point(endPos)))
                .setLinearHeadingInterpolation(interPos.getHeading(), endPos.getHeading())
                .addPath(new BezierLine(new Point(endPos), new Point(startPos)))
                .setLinearHeadingInterpolation(endPos.getHeading(), startPos.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(path, true);
                    pivotExtension.pivotTarget = 90;
                    pivotExtension.liftTarget = 300;
                    setPathState(-1);
                }
                break;
            default:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    pivotExtension.pivotTarget = 45;
                    pivotExtension.liftTarget = 500;
                    requestOpModeStop();
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        slides.update();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        slides = new pivotExtension(hardwareMap, telemetry);
        follower.setStartingPose(startPos);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

}
