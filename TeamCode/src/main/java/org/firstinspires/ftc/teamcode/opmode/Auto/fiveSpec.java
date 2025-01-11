package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.teamcode.config.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "fiveSpec", group = "auto")
public class fiveSpec extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer;
    private EndEffector endEffector;
    private Deposit deposit;
    private int pathState;

    private PathChain preload, pushSample1Path, pushSample2Path, pushSample3Path, score1, return1, score2, return2, score3, return3, score4, return4;
   private Pose startingPose =  new Pose   (7, 65, Math.toRadians(0));


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(7.338, 65.859, Math.toRadians(0))),
                        new Point(new Pose(40, 65.859, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Push Sample 1 Path (Lines 2 and 3)
        pushSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(55, 30.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(55, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(13.250, 24.000, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Push Sample 2 Path (Lines 4 and 5)
        pushSample2Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(13.250, 24.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 22.000, Math.toRadians(0))),
                        new Point(new Pose(55, 12.500, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(55, 12.500, Math.toRadians(0))),
                        new Point(new Pose(13.250, 12.500, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Push Sample 3 Path (Lines 6 and 7)
        pushSample3Path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(13.250, 12.500, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 8, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 8, Math.toRadians(0))),
                        new Point(new Pose(8.5, 8, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(8.5, 8, Math.toRadians(0))),
                        new Point(new Pose(36, 24, Math.toRadians(0))),
                        new Point(new Pose(12, 48, Math.toRadians(0))),
                        new Point(new Pose(40, 68, Math.toRadians(0)))))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, true);

                }

                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                    deposit.setSlideTarget(500);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                    deposit.setSlideTarget(0);
                    deposit.setPivotTarget(90);
                    endEffector.setWallIntakePositionAlt();
                    follower.followPath(pushSample1Path,false);
                    setPathState(2);
            }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pushSample2Path, false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(pushSample3Path, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    endEffector.closeClaw();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1, true);
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
                    endEffector.setSpecScore();
                    deposit.setSlideTarget(100);
                }


                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                    deposit.setSlideTarget(430);
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    deposit.setSlideTarget(230);
                    if (deposit.liftPos < 250) {
                        endEffector.openClaw();
                        setPathState(-1);
                    }
                }
                break;
            default:
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
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());


        telemetry.update();

        deposit.update();

        autonomousPathUpdate();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();

        // Initialize follower, slides, etc. as usual
        follower = new Follower(hardwareMap);

        // If needed, set a starting pose (only if your system requires it)
        // follower.setStartingPose(new Pose(0,0, 0));
        follower.setStartingPose(startingPose);

        deposit = new Deposit(hardwareMap, telemetry, true);
        endEffector = new EndEffector(hardwareMap);

        endEffector.setSpecScore();
        deposit.setPivotTarget(90);
        deposit.setSlideTarget(50);

        // Build our newly incorporated multi-step path:
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}