package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

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
    private int pathState;

    // Remove or comment out old pose references if they are not used anywhere else:
    // private Pose startPos = new Pose(0,0, Math.toRadians(0));
    // private Pose interPos = new Pose(24, -24, Math.toRadians(90));
    // private Pose endPos = new Pose(24, 24, Math.toRadians(45));
    private PathChain preload;
    private PathChain pushSample1Path;
    private PathChain pushSample2Path;
   private Pose startingPose =  new Pose   (7, 65, Math.toRadians(0));
   private Pose subPose =  new Pose   (35, 65, Math.toRadians(0));
    private Pose behindSample1ControlPoint1 = new Pose(1.4,30,Math.toRadians(0));
    private Pose behindSample1ControlPoint2 = new Pose(82,45,Math.toRadians(0)); // x og 102
   private Pose behindSample1 = new Pose (53,23,Math.toRadians(0)); // x og 53

   private Pose pushSample1 = new Pose (25, 23, Math.toRadians(0));

   private Pose behindSample2ControlPoint = new Pose(91,24,Math.toRadians(0));
   private Pose behindSample2 = new Pose (54,12,Math.toRadians(0));
   private Pose pushSample2 = new Pose (7, 12, Math.toRadians(0));
   private Pose behindSample3ControlPoint = new Pose(100,13,Math.toRadians(0));
   private Pose behindSample3 = new Pose (53,3,Math.toRadians(0));
   private Pose pushSample3 = new Pose (7, 3, Math.toRadians(0));


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Build your path exactly as in your GeneratedPath class:
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(subPose)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), subPose.getHeading())
                .build();
        pushSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose), new Point(behindSample1ControlPoint1), new Point(behindSample1ControlPoint2), new Point(behindSample1)))
                .setLinearHeadingInterpolation(subPose.getHeading(), behindSample1.getHeading())
                .addPath(new BezierLine(new Point(behindSample1), new Point(pushSample1)))
                .setLinearHeadingInterpolation(behindSample1.getHeading(), pushSample1.getHeading())
              //  .addPath(new BezierCurve(new Point(pushSample1), new Point(behindSample2ControlPoint), new Point(behindSample2)))
               // .setLinearHeadingInterpolation(pushSample1.getHeading(), behindSample2.getHeading())
//                .addPath(new BezierLine(new Point(behindSample2), new Point(pushSample2)))
//                .setLinearHeadingInterpolation(behindSample2.getHeading(), pushSample2.getHeading())
//                .addPath(new BezierCurve(new Point(pushSample2), new Point(behindSample3ControlPoint), new Point(behindSample3)))
//                .setLinearHeadingInterpolation(pushSample2.getHeading(), behindSample3.getHeading())
//                .addPath(new BezierLine(new Point(behindSample3), new Point(pushSample3)))
//                .setLinearHeadingInterpolation(behindSample3.getHeading(), pushSample3.getHeading())
                .build();

        pushSample2Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSample1), new Point(behindSample2ControlPoint), new Point(behindSample2)))
                 .setLinearHeadingInterpolation(pushSample1.getHeading(), behindSample2.getHeading())
                .addPath(new BezierLine(new Point(behindSample2), new Point(pushSample2)))
                .setLinearHeadingInterpolation(behindSample2.getHeading(), pushSample2.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, true);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pushSample1Path,true);
                    setPathState(2);
            }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(pushSample2Path, false);
                    setPathState(-1);
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

        telemetry.update();

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


        // Build our newly incorporated multi-step path:
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }
}