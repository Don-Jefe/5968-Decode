package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Forward 2ft Left 1ft", group = "Examples")
public class AutoSigma extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Starting pose at origin, facing 0°
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Target pose: 24 inches forward, 12 inches left, facing 30°
    private final Pose targetPose = new Pose(24, 12, Math.toRadians(30));

    private Path movePath;

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build path from startPose to targetPose
        movePath = new Path(new BezierLine(startPose, targetPose));
        movePath.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState == 0) {
            follower.followPath(movePath);
            pathState = 1;
        } else if (pathState == 1 && !follower.isBusy()) {
            pathState = -1; // End of sequence
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {}
}
