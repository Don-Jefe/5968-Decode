package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.PanelsTelemetry.*;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "far", group = "Autonomous")
@Configurable
public class FarAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    public boolean isBlueAlliance = true;
    private Timer pathTimer;
    private Drivetrain drivetrain;
    private Paths paths;
    private int step = 0;
    private long stepTimer = 0;

    private boolean waitMs(long ms) {
        if (stepTimer == 0) {
            stepTimer = System.currentTimeMillis();
            return false;
        }
        if (System.currentTimeMillis() - stepTimer >= ms) {
            stepTimer = 0;
            return true;
        }
        return false;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        drivetrain = new Drivetrain(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        drivetrain.blocker.setTargetPosition(0);
        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.blocker.setPower(0.5);
        // drivetrain.feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        pathTimer = new Timer();


        follower.setMaxPower(0.8);
//        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drivetrain.blocker.setPower(1);
//        if(waitMs(500)) {
//            drivetrain.blocker.setPower(0);
//        }
//        // Reset encoder at closed position = 0
//        drivetrain.blocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Enable RUN_TO_POSITION mode
//        drivetrain.blocker.setTargetPosition(170);
//        drivetrain.blocker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drivetrain.blocker.setPower(0.5);
//        if(waitMs(500)){
//            drivetrain.blocker.setPower(0);
//        }



        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            isBlueAlliance = false;  // RED ALLIANCE
        } else if (gamepad1.b) {
            isBlueAlliance = true;   // BLUE ALLIANCE
        }

        telemetry.addData("Alliance Selected",
                isBlueAlliance ? "Blue" : "Red");
        telemetry.update();
    }

    @Override
    public void start() {
        // build paths after alliance selection
        paths = new Paths(follower, isBlueAlliance);
        if (isBlueAlliance) {
            follower.setStartingPose(new Pose(00, 00, Math.toRadians(325)));
        } else {
            follower.setStartingPose(new Pose(00, 00, Math.toRadians(325)));
        }

        // reset step timer
        step = 0;
        stepTimer = 0;
    }

    @Override
    public void loop() {
        pathTimer.resetTimer();
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", step);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    private static class SleepHolder {
        static long start = 0;
        static boolean running = false;
    }

    public boolean sleepNonBlocking(long ms) {
        if (!SleepHolder.running) {
            SleepHolder.start = System.currentTimeMillis();
            SleepHolder.running = true;
            return false;   // sleep just started
        }

        if (System.currentTimeMillis() - SleepHolder.start >= ms) {
            SleepHolder.running = false;
            return true;    // sleep finished
        }

        return false;       // still waiting
    }

    // -------------------------------------------------------------
    // PATH DEFINITIONS (UNCHANGED)
    // -------------------------------------------------------------
    public static class Paths {

        public PathChain ShootingPosition1;
        public PathChain BeginPickup1;
        public PathChain EndPickup1;
        public PathChain ShootingPosition2;
        public PathChain BeginPickup2;
        public PathChain EndPickup2;
        public PathChain ShootingPosition3;
        public PathChain BeginPickup3;
        public PathChain EndPickup3;
        public PathChain ShootingPosition4;
        public PathChain Park;

        public Paths(Follower follower, boolean isBlue) {
            if (isBlue) {
                ShootingPosition1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(0, 0), new Pose(-24, 0))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(325))
                        .build();

                BeginPickup1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(58, 92), new Pose(50.479, 82.815))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                        .build();

                EndPickup1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(50.479, 82.815), new Pose(26.357, 82.815))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                ShootingPosition2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(26.357, 82.815), new Pose(60.145, 90.896))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(325))
                        .build();

                BeginPickup2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(60.145, 90.896), new Pose(42.284, 58.263))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                        .build();

                EndPickup2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.284, 58.263), new Pose(30.747, 58.263))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                ShootingPosition3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(30.747, 58.263), new Pose(60.340, 94.896))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(325))
                        .build();

                BeginPickup3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(60.340, 94.896), new Pose(41.894, 33.516))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(180))
                        .build();

                EndPickup3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(41.894, 33.516), new Pose(28.162, 33.710))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                ShootingPosition4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(28.162, 33.710), new Pose(55.340, 94.896))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(325))
                        .build();

                Park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(55.340, 94.896), new Pose(53.391, 51.637))
                        )
                        .setTangentHeadingInterpolation()
                        .build();
            } else {
                ShootingPosition1  = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(123, 123), new Pose(84.567, 86.701))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(220))
                        .build();

                BeginPickup1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(84.567, 86.701), new Pose(104.946, 82.620))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                        .build();

                EndPickup1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(104.946, 82.620), new Pose(117.795, 82.230))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                ShootingPosition2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(117.795, 82.230), new Pose(84.762, 86.701))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                        .build();

                BeginPickup2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(84.762, 86.701), new Pose(103.479, 57.483))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                        .build();

                EndPickup2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(103.479, 57.483), new Pose(117.575, 57.288))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                ShootingPosition3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(117.575, 57.288), new Pose(84.762, 86.701))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                        .build();

                BeginPickup3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(84.762, 86.701), new Pose(103.089, 32.736))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(0))
                        .build();

                EndPickup3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(103.089, 32.736), new Pose(117.380, 32.736))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                ShootingPosition4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(117.380, 32.736), new Pose(84.762, 86.701))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(220))
                        .build();

                Park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(92.762, 94.701), new Pose(93.736, 58.068))
                        )
                        .setTangentHeadingInterpolation()
                        .build();
            }
        }
    }

    // -------------------------------------------------------------
    // AUTONOMOUS FSM (UNCHANGED)
    // -------------------------------------------------------------
    private void autonomousPathUpdate() {
        drivetrain.setFlywheelRPM(-4540);

        switch (step) {

            // ----------------------------------------------------
            // DRIVE PATH 1
            // ----------------------------------------------------
            case 0:

                if(waitMs(5000)) {
                    drivetrain.setFeederPower(1.00);
                    drivetrain.setIntakePower(-1.0);
                    step++;
                    break;
                }

            case 1:  // wait until done
                if(waitMs(3000)) {
                    drivetrain.setFeederPower(1.00);
                    drivetrain.setIntakePower(-1.0);
                    step++;
                }

            // ----------------------------------------------------
            // SPIN-UP + FEED FOR 2 SECONDS
            // ----------------------------------------------------
            case 2:

                if (waitMs(15000)) {
                    drivetrain.setFeederPower(0);
                    drivetrain.closeShoot();
                    follower.followPath(paths.ShootingPosition1);
                    step++;
                }
                break;


        }
    }

}

