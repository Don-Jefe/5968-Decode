package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name = "Blue Shoot Tuah", group = "Examples")
public class AutoSigma extends OpMode {

    private boolean isRedAlliance = false;
    private Drivetrain drivetrain;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    private final Pose startPose = new Pose(85, 0, Math.toRadians(0));
    private final Pose targetPose = new Pose(0, 0, Math.toRadians(45));
    private final Pose endPose = new Pose(50, 5, Math.toRadians(0));

    private Path movePath;
    private Path lastPath;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        follower.setStartingPose(startPose);

        // === Move VisionPortal setup here ===
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640, 360))
                .enableLiveView(true)
                .build();
        // === ============================ ===

        // Path 1
        movePath = new Path(new BezierLine(startPose, targetPose));
        movePath.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());

        // Path 2
        lastPath = new Path(new BezierLine(targetPose, endPose));
        lastPath.setLinearHeadingInterpolation(targetPose.getHeading(), endPose.getHeading());
    }
    @Override
    public void init_loop() {

        // Determine alliance from gamepad input
        if (gamepad1.a) {
            isRedAlliance = false; // Blue
        } else if (gamepad1.b) {
            isRedAlliance = true; // Red
        }

        // Update telemetry
        telemetry.addData("Alliance Selected", isRedAlliance ? "Red" : "Blue");
        telemetry.update();
    }


    @Override
    public void start() {
        pathTimer.resetTimer();
        pathState = 0;

//        if (processor.getDetections().size() > 0) {
//            AprilTagDetection tag = processor.getDetections().get(0);
//        }

    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(movePath);
                pathState = -1;
//                drivetrain.setFlywheelPower(-.55);

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
                if (pathTimer.getElapsedTimeSeconds() < 3) {
//                    drivetrain.setFlywheelPower(-0.55);
                    // Feed balls for 2 seconds after spin-up (3s → 5s)
                }
                else if (pathTimer.getElapsedTimeSeconds() < 10) {
//                    drivetrain.setFlywheelPower(-0.55);
//                    drivetrain.setFeederPower(1);
                    drivetrain.intakeOut();
                }
                // Stop shooter after 5 seconds
                else {
//                    drivetrain.setFlywheelPower(0);
//                    drivetrain.setFeederPower(0);
                    drivetrain.intakeStop();

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
//        drivetrain.setFeederPower(0);
//        drivetrain.setFlywheelPower(0);
        drivetrain.setIntakePower(0);
    }
}
