//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry.*;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Shoot 12-uh", group = "Autonomous")
//@Configurable
//public class DecodeAuton extends OpMode {
//
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    public boolean isBlueAlliance = true;
//    private Timer pathTimer;
//    private Drivetrain drivetrain;
//    private Paths paths;
//    private int step = 0;
//    private long stepTimer = 0;
//
//    private boolean waitMs(long ms) {
//        if (stepTimer == 0) {
//            stepTimer = System.currentTimeMillis();
//            return false;
//        }
//        if (System.currentTimeMillis() - stepTimer >= ms) {
//            stepTimer = 0;
//            return true;
//        }
//        return false;
//    }
//
//    @Override
//    public void init() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        drivetrain = new Drivetrain(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//
//        pathTimer = new Timer();
//
//        follower.setMaxPower(0.8);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//    @Override
//    public void init_loop() {
//        if (gamepad1.a) {
//            isBlueAlliance = false;  // RED ALLIANCE
//        } else if (gamepad1.b) {
//            isBlueAlliance = true;   // BLUE ALLIANCE
//        }
//
//        telemetry.addData("Alliance Selected",
//                isBlueAlliance ? "Blue" : "Red");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        // build paths after alliance selection
//        paths = new Paths(follower, isBlueAlliance);
//        if (isBlueAlliance) {
//            follower.setStartingPose(new Pose(21, 123, Math.toRadians(325)));
//        } else {
//            follower.setStartingPose(new Pose(123, 123, Math.toRadians(35+180)));
//        }
//        // reset step timer
//        step = 0;
//        stepTimer = 0;
//    }
//
//    @Override
//    public void loop() {
//        pathTimer.resetTimer();
//        follower.update();
//        autonomousPathUpdate();
//
//        panelsTelemetry.debug("Path State", step);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
//        panelsTelemetry.update(telemetry);
//    }
//
//
//    private static class SleepHolder {
//        static long start = 0;
//        static boolean running = false;
//    }
//
//    public boolean sleepNonBlocking(long ms) {
//        if (!SleepHolder.running) {
//            SleepHolder.start = System.currentTimeMillis();
//            SleepHolder.running = true;
//            return false;   // sleep just started
//        }
//
//        if (System.currentTimeMillis() - SleepHolder.start >= ms) {
//            SleepHolder.running = false;
//            return true;    // sleep finished
//        }
//
//        return false;       // still waiting
//    }
//
//    // -------------------------------------------------------------
//    // PATH DEFINITIONS (UNCHANGED)
//    // -------------------------------------------------------------
//    public static class Paths {
//
//        public PathChain ShootingPosition1;
//        public PathChain BeginPickup1;
//        public PathChain EndPickup1;
//        public PathChain ShootingPosition2;
//        public PathChain BeginPickup2;
//        public PathChain EndPickup2;
//        public PathChain ShootingPosition3;
//        public PathChain BeginPickup3;
//        public PathChain EndPickup3;
//        public PathChain ShootingPosition4;
//        public PathChain Park;
//        public PathChain Backup;
//
//        public Paths(Follower follower, boolean isBlue) {
//            if (isBlue) {
//                ShootingPosition1 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.S1_START, CF.S1_END))
//                        .setLinearHeadingInterpolation(CF.S1_HEAD_START, CF.S1_HEAD_END)
//                        .build();
//
//                BeginPickup1 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P1_START, CF.P1_DROP))
//                        .setLinearHeadingInterpolation(CF.P1_HEAD_START, CF.P1_HEAD_DOWN)
//                        .build();
//
//                EndPickup1 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P1_DROP, CF.P1_END))
//                        .setLinearHeadingInterpolation(CF.P1_HEAD_DOWN, CF.P1_HEAD_END)
//                        .build();
//
//                ShootingPosition2 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.S2_START, CF.S2_END))
//                        .setLinearHeadingInterpolation(CF.S2_HEAD_START, CF.S2_HEAD_END)
//                        .build();
//
//                BeginPickup2 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P2_START, CF.P2_DROP))
//                        .setLinearHeadingInterpolation(CF.P2_HEAD_START, CF.P2_HEAD_DOWN)
//                        .build();
//
//                EndPickup2 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P2_DROP, CF.P2_END))
//                        .setLinearHeadingInterpolation(CF.P2_HEAD_DOWN, CF.P2_HEAD_END)
//                        .build();
//
//                Backup = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.BACKUP_START, CF.BACKUP_END))
//                        .setLinearHeadingInterpolation(CF.BACKUP_HEAD_START, CF.BACKUP_HEAD_END)
//                        .build();
//
//                ShootingPosition3 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.S3_START, CF.S3_END))
//                        .setLinearHeadingInterpolation(CF.S3_HEAD_START, CF.S3_HEAD_END)
//                        .build();
//
//                BeginPickup3 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P3_START, CF.P3_DROP))
//                        .setLinearHeadingInterpolation(CF.P3_HEAD_START, CF.P3_HEAD_DOWN)
//                        .build();
//
//                EndPickup3 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.P3_DROP, CF.P3_END))
//                        .setLinearHeadingInterpolation(CF.P3_HEAD_DOWN, CF.P3_HEAD_END)
//                        .build();
//
//                ShootingPosition4 = follower.pathBuilder()
//                        .addPath(new BezierLine(CF.S4_START, CF.S4_END))
//                        .setLinearHeadingInterpolation(CF.S4_HEAD_START, CF.S4_HEAD_END)
//                        .build();
//
//
//            } else {
//               /* make a new red side auton ASAP
//               *  sigmmagmmassg
//               * a
//               * */
//            }
//        }
//    }
//
//    // -------------------------------------------------------------
//    // AUTONOMOUS FSM (UNCHANGED)
//    // -------------------------------------------------------------
//    private void autonomousPathUpdate() {
//        drivetrain.setFlywheelRPM(-3340);
//
//        switch (step) {
//
//            // ----------------------------------------------------
//            // DRIVE PATH 1
//            // ----------------------------------------------------
//            case 0:
//
//
//                follower.followPath(paths.ShootingPosition1);
//                step++;
//                break;
//
//            case 1:  // wait until done
//                if (!follower.isBusy()) step++;
//                //drivetrain.setFlywheelPower(-0.77);
//                drivetrain.openShoot();
//                break;
//
//            // ----------------------------------------------------
//            // SPIN-UP + FEED FOR 2 SECONDS
//            // ----------------------------------------------------
//            case 2:
//                drivetrain.setFeederPower(1.00);
//                drivetrain.setIntakePower(-1.0);
//                if (waitMs(3000)) {
//                    drivetrain.setFeederPower(0);
//                    drivetrain.closeShoot();
//                    step++;
//                }
//                break;
//
//            // ----------------------------------------------------
//            // DRIVE PATH 2
//            // ----------------------------------------------------
//            case 3:
//                follower.followPath(paths.BeginPickup1);
//                step++;
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    drivetrain.closeShoot();
//                    step++;
//                }
//
//                break;
//
//            // ----------------------------------------------------
//            // TURN ON INTAKE + FEED, DRIVE PATH 3
//            // ----------------------------------------------------
//            case 5:
//                drivetrain.closeShoot();
//                drivetrain.setIntakePower(-1.0);
//                drivetrain.setFeederPower(1);
//                follower.followPath(paths.EndPickup1);
//                step++;
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    drivetrain.openShoot();
//                    drivetrain.setFeederPower(0);
//                    //  drivetrain.setFlywheelPower(-.87);
//                    step++;
//                }
//                break;
//
//            // ----------------------------------------------------
//            // DRIVE PATH 4 → PATH 5
//            // ----------------------------------------------------
//            case 7:
//
//                follower.followPath(paths.ShootingPosition1);
//                step++;
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    drivetrain.openShoot();
//                    drivetrain.setFeederPower(1);
//                    drivetrain.setIntakePower(-1.0);
//                    if (waitMs(2500)) {
//                        drivetrain.setFeederPower(0);
//                        // drivetrain.setFlywheelPower(0);
//                        drivetrain.closeShoot();
//                        step++;
//                    }
//                }
//                break;
//
//            case 9:
//                follower.followPath(paths.BeginPickup2);
//                step++;
//                break;
//
//            case 10:
//                if (!follower.isBusy()) step++;
//                break;
//
//            // ----------------------------------------------------
//            // TURN ON INTAKE + FEED, DRIVE PATH 3
//            // ----------------------------------------------------
//            case 11:
//                drivetrain.setIntakePower(-1.0);
//                drivetrain.setFeederPower(1);
//                follower.followPath(paths.EndPickup2);
//                step++;
//                break;
//
//            case 12:
//                if (!follower.isBusy()) {
//                    drivetrain.setFeederPower(0);
//                    step++;
//                }
//                break;
//
//            // ----------------------------------------------------
//            // DRIVE PATH 4 → PATH 5
//            // ----------------------------------------------------
//            case 13:
//                drivetrain.openShoot();
//                follower.followPath(paths.ShootingPosition2);
//                step++;
//                break;
//
//            case 14:
//                if (!follower.isBusy()) {
//                    drivetrain.setFeederPower(1);
//                    drivetrain.setIntakePower(-1.0);
//                    if (waitMs(2500)) {
//                        drivetrain.setFeederPower(0);
//                        //   drivetrain.setFlywheelPower(0);
//                        drivetrain.closeShoot();
//                        step++;
//                    }
//                }
//                break;
//            case 15:
//                follower.followPath(paths.BeginPickup3);
//                step++;
//                break;
//
//            case 16:
//                if (!follower.isBusy()) step++;
//                break;
//
//            // ----------------------------------------------------
//            // TURN ON INTAKE + FEED, DRIVE PATH 3
//            // ----------------------------------------------------
//            case 17:
//                drivetrain.setIntakePower(-1.0);
//                drivetrain.setFeederPower(1);
//                follower.followPath(paths.EndPickup3);
//                step++;
//                break;
//
//            case 18:
//                if (!follower.isBusy()) {
//                    drivetrain.setFeederPower(0);
//                    step++;
//                }
//                break;
//
//            // ----------------------------------------------------
//            // DRIVE PATH 4 → PATH 5
//            // ----------------------------------------------------
//            case 19:
//                drivetrain.openShoot();
//                follower.followPath(paths.ShootingPosition3);
//                step++;
//                break;
//
//            case 20:
//                if (!follower.isBusy()) {
//                    drivetrain.setFeederPower(1);
//                    drivetrain.setIntakePower(-1.0);
//
//                    if (waitMs(2500)) {
//                        drivetrain.setFeederPower(0);
//                        //drivetrain.setFlywheelPower(0);
//                        drivetrain.closeShoot();
//                        step++;
//                    }
//                }
//                break;
//            case 21:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Park);
//                }
//        }
//    }
//
//}