package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Forward, Shoot, Back 30in", group = "Examples")
public class AutoSigma extends OpMode {

    private Drivetrain drivetrain;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Starting pose at origin, facing 0°
    private final Pose startPose = new Pose(80, 0, Math.toRadians(0));

    // Move forward to shooting position
    private final Pose targetPose = new Pose(0, 0, Math.toRadians(45));

    // Move backward slightly after shooting
    private final Pose endPose = new Pose(50, 0, Math.toRadians(0));

    private Path movePath;
    private Path lastPath;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        follower.setStartingPose(startPose);

        // Path 1: drive to shooting position
        movePath = new Path(new BezierLine(startPose, targetPose));
        movePath.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());

        // Path 2: move back 30 inches
        lastPath = new Path(new BezierLine(targetPose, endPose));
        lastPath.setLinearHeadingInterpolation(targetPose.getHeading(), endPose.getHeading());
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(movePath);
                pathState = 1;
                drivetrain.setFlywheelPower(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    // Start shooting sequence timer
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                // Flywheel spins up for 3 seconds
                if (pathTimer.getElapsedTimeSeconds() < 5.0) {

                }
                // Feed balls for 2 seconds after spin-up (3s → 5s)
                else if (pathTimer.getElapsedTimeSeconds() < 10.0) {
                    drivetrain.setFlywheelPower(1);
                    drivetrain.setFeederPower(1);
                }
                // Stop shooter after 5 seconds
                else {
                    drivetrain.setFlywheelPower(0);
                    drivetrain.setFeederPower(0);

                    // Start next movement
                    follower.followPath(lastPath);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = -1; // Done
                }
                break;
        }

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Timer (s)", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.setFeederPower(0);
        drivetrain.setFlywheelPower(0);
        drivetrain.setIntakePower(0);
    }
}
