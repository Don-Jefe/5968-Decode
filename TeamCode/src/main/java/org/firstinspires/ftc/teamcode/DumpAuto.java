package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "VibeCodeTele")
public class DumpAuto extends OpMode {

    double maxp = 0.9;

    private enum AutoState {
        PRELOAD,

        SHOOT_1,
        TO_PICK_1_START,
        PICK_1,

        SHOOT_2,
        TO_PICK_2_START,
        PICK_2,

        DUMP_WAIT,

        SHOOT_3,
        PARK,
        DONE
    }

    private AutoState state;

    private Follower follower;
    private Drivetrain drivetrain;
    private Paths paths;
    private TelemetryManager telemetryM;
    private final Timer stateTimer = new Timer();

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117, 127.625, Math.toRadians(45)));
        follower.update();

        paths = new Paths(follower);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.followPath(paths.Shooting1);
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        state = AutoState.PRELOAD;
        stateTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // keep flywheel constantly regulated
        drivetrain.NewSetFlywheelRPM(-2850, 18, 0, 0, 14);

        switch (state) {
            case PRELOAD:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 2.5) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_1;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_1:
                if (stateTimer.getElapsedTimeSeconds() > 1.5) {
                    setPickupState();
                    follower.followPath(paths.PickStart1);
                    state = AutoState.TO_PICK_1_START;
                }
                break;

            case TO_PICK_1_START:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.4);
                    follower.followPath(paths.PickEnd1);
                    state = AutoState.PICK_1;
                }
                break;

            case PICK_1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(maxp);
                    setShootingState();
                    follower.followPath(paths.Shooting2);
                    state = AutoState.SHOOT_2;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_2:
                drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                if (stateTimer.getElapsedTimeSeconds() > 1.5) {
                    setPickupState();
                    follower.followPath(paths.PickStart2);
                    state = AutoState.TO_PICK_2_START;
                }
                break;

            case TO_PICK_2_START:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.4);
                    follower.followPath(paths.PickEnd2);
                    state = AutoState.PICK_2;
                }
                break;

            case PICK_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(maxp);
                    setIdleState();
                    follower.followPath(paths.Dump);
                    state = AutoState.DUMP_WAIT;
                    stateTimer.resetTimer();
                }
                break;

            case DUMP_WAIT:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 5.0) {
                    setShootingState();
                    follower.followPath(paths.Shooting3);
                    state = AutoState.SHOOT_3;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_3:
                drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                if (stateTimer.getElapsedTimeSeconds() > 1.5) {
                    setPickupState();
                    follower.followPath(paths.END);
                    state = AutoState.PARK;
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                drivetrain.setIntakePower(0);
                drivetrain.setFeederPower(0);
                break;
        }

        telemetryM.debug("State", state);
        telemetryM.debug("Time", stateTimer.getElapsedTimeSeconds());
    }

    private void setShootingState() {
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(1.0);
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-.8);
        drivetrain.setFeederPower(0.6);
    }

    private void setIdleState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(0);
        drivetrain.setFeederPower(0);
    }

    /*
     ================= PATHS =================
     */
    public static class Paths {
        public PathChain Shooting1;
        public PathChain PickStart1;
        public PathChain PickEnd1;
        public PathChain Shooting2;
        public PathChain PickStart2;
        public PathChain PickEnd2;
        public PathChain Dump;
        public PathChain Shooting3;
        public PathChain END;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(117.041, 127.625),
                            new Pose(87.792, 87.056)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45)).build();

            PickStart1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87.792, 87.056),
                            new Pose(97.187, 83.658)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

            PickEnd1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(97.187, 83.658),
                            new Pose(120.046, 83.394)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Shooting2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(120.046, 83.394),
                            new Pose(87.263, 86.721)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45)).build();

            PickStart2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87.263, 86.721),
                            new Pose(96.007, 59.614)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)).build();

            PickEnd2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(96.007, 59.614),
                            new Pose(120.027, 59.826)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Dump = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(120.027, 59.826),
                            new Pose(76.765, 62.944),
                            new Pose(128.903, 70.274)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180)).build();

            Shooting3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(128.903, 70.274),
                            new Pose(87, 87)
                    )
            ).setTangentHeadingInterpolation().build();

            END = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(87, 87),
                            new Pose(95, 120)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
        }
    }
}
