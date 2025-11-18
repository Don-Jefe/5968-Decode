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

@Autonomous(name = "Red+ Blue Alliance Auto with Tag Detection", group = "Examples")
public class CaseyBarker extends OpMode {

    private boolean isRedAlliance = false; // Default to blue
    private int detectedTagId = -1; // Store the detected AprilTag ID (-1 if none)
    private Drivetrain drivetrain;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private VisionPortal visionPortal;
    private AprilTagProcessor processor;

    // Poses will be set based on alliance
    private Pose startPose;
    private Pose targetPose;
    private Pose endPose;

    private Path movePath;
    private Path lastPath;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();

        // Set up vision
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
                .build();

        // Initial telemetry
        telemetry.addData("Alliance Selection", "Press A for Blue, B for Red");
        telemetry.addData("Detected Tag ID", detectedTagId);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Check for AprilTag detections and store the ID
        if (!processor.getDetections().isEmpty()) {
            AprilTagDetection tag = processor.getDetections().get(0); // Take the first detected tag
            detectedTagId = tag.id;
        }

        // Determine alliance from gamepad input
        if (gamepad1.a) {
            isRedAlliance = false; // Blue
        } else if (gamepad1.b) {
            isRedAlliance = true; // Red
        }

        // Update telemetry
        telemetry.addData("Alliance Selected", isRedAlliance ? "Red" : "Blue");
        telemetry.addData("Detected Tag ID", detectedTagId);
        telemetry.update();
    }

    @Override
    public void start() {
        // Set poses based on selected alliance
        if (isRedAlliance) {
            startPose = new Pose(85, 0, Math.toRadians(0));
            targetPose = new Pose(0, 0, Math.toRadians(-45));
            endPose = new Pose(50, 0, Math.toRadians(0));
        } else {
            startPose = new Pose(85, 0, Math.toRadians(0));
            targetPose = new Pose(0, 0, Math.toRadians(45));
            endPose = new Pose(50, 5, Math.toRadians(0));
        }

        follower.setStartingPose(startPose);

        // Create paths
        movePath = new Path(new BezierLine(startPose, targetPose));
        movePath.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());

        lastPath = new Path(new BezierLine(targetPose, endPose));
        lastPath.setLinearHeadingInterpolation(targetPose.getHeading(), endPose.getHeading());

        pathTimer.resetTimer();
        pathState = 0;

        // Close vision portal to save resources
        visionPortal.close();
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(movePath);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                // Flywheel spins up for 3 seconds (commented out as in original)
                if (pathTimer.getElapsedTimeSeconds() < 3) {
                    // drivetrain.setFlywheelPower(-0.55);
                }
                // Feed balls for next 7 seconds
                else if (pathTimer.getElapsedTimeSeconds() < 10) {
                    // drivetrain.setFlywheelPower(-0.55);
                    // drivetrain.setFeederPower(1);
                    drivetrain.intakeOut();
                }
                // Stop shooter
                else {
                    // drivetrain.setFlywheelPower(0);
                    // drivetrain.setFeederPower(0);
                    drivetrain.intakeStop();
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
        telemetry.addData("Alliance", isRedAlliance ? "Red" : "Blue");
        telemetry.addData("Stored Tag ID", detectedTagId);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Timer (s)", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.setIntakePower(0);
        // drivetrain.setFeederPower(0);
        // drivetrain.setFlywheelPower(0);
    }
}
