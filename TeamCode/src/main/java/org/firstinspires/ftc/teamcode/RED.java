package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Big Red Booti")
public class RED extends OpMode {

    /* =========================
       STATE MACHINE
       ========================= */
    private enum AutoState {
        SHOOT_1,
        TO_PICK_1_START,
        PICK_1,

        SHOOT_2,
        TO_PICK_2_START,
        PICK_2,

        SHOOT_3,
        TO_PICK_3_START,
        PICK_3,

        SHOOT_4,
        PARK,
        DONE
    }

    private AutoState state;

    /* =========================
       HARDWARE
       ========================= */
    private Follower follower;
    private Drivetrain drivetrain;
    private Paths paths;
    private TelemetryManager telemetryM;
    private final Timer stateTimer = new Timer();

    /* =========================
       INIT
       ========================= */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(117.041, 127.625, Math.toRadians(45))
        );
        follower.update();

        paths = new Paths(follower);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        drivetrain.setFlywheelPower(1.0);

        setShootingState();
        follower.followPath(paths.Shooting1);

        state = AutoState.SHOOT_1;
        stateTimer.resetTimer();
    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        drivetrain.setFlywheelRPM(-2700);

        switch (state) {

            case SHOOT_1:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
                    setIdleState();
                    follower.followPath(paths.PickStart1);
                    state = AutoState.TO_PICK_1_START;
                }
                break;

            case TO_PICK_1_START:
                if (!follower.isBusy()) {
                    setPickupState();
                    follower.followPath(paths.PickEnd1);
                    state = AutoState.PICK_1;
                }
                break;

            case PICK_1:
                if (!follower.isBusy()) {
                    setShootingState();
                    follower.followPath(paths.Shooting2);
                    state = AutoState.SHOOT_2;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_2:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
                    setIdleState();
                    follower.followPath(paths.PickStart2);
                    state = AutoState.TO_PICK_2_START;
                }
                break;

            case TO_PICK_2_START:
                if (!follower.isBusy()) {
                    setPickupState();
                    follower.followPath(paths.PickEnd2);
                    state = AutoState.PICK_2;
                }
                break;

            case PICK_2:
                if (!follower.isBusy()) {
                    setShootingState();
                    follower.followPath(paths.Shooting3);
                    state = AutoState.SHOOT_3;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_3:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
                    setIdleState();
                    follower.followPath(paths.PickStart3);
                    state = AutoState.TO_PICK_3_START;
                }
                break;

            case TO_PICK_3_START:
                if (!follower.isBusy()) {
                    setPickupState();
                    follower.followPath(paths.PickEnd3);
                    state = AutoState.PICK_3;
                }
                break;

            case PICK_3:
                if (!follower.isBusy()) {
                    setShootingState();
                    follower.followPath(paths.Shooting4);
                    state = AutoState.SHOOT_4;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_4:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
                    setIdleState();
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

        telemetryM.debug("Auto State", state);
        telemetryM.debug("State Time", stateTimer.getElapsedTimeSeconds());
    }

    /* =========================
       MECHANISM STATES
       ========================= */
    private void setShootingState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(1.0);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(0.6);
    }

    private void setIdleState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(0);
        drivetrain.setFeederPower(0);
    }

    /* =========================
       PATH DEFINITIONS (RED)
       ========================= */
    public static class Paths {

        public PathChain Shooting1;

        public PathChain PickStart1;
        public PathChain PickEnd1;

        public PathChain Shooting2;

        public PathChain PickStart2;
        public PathChain PickEnd2;

        public PathChain Shooting3;

        public PathChain PickStart3;
        public PathChain PickEnd3;

        public PathChain Shooting4;

        public PathChain END;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117.041, 127.625),
                                    new Pose(87.792, 87.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                    .build();

            PickStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.792, 87.056),
                                    new Pose(97.187, 83.658)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.187, 83.658),
                                    new Pose(120.046, 83.394)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.046, 83.394),
                                    new Pose(87.263, 86.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            PickStart2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.263, 86.721),
                                    new Pose(96.007, 59.614)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .setReversed()
                    .build();

            PickEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.007, 59.614),
                                    new Pose(120.027, 59.826)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.027, 59.826),
                                    new Pose(87.365, 86.983)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .setReversed()
                    .build();

            PickStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.365, 86.983),
                                    new Pose(96.693, 34.913)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .setReversed()
                    .build();

            PickEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.693, 34.913),
                                    new Pose(119.268, 34.970)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setReversed()
                    .build();

            Shooting4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(119.268, 34.970),
                                    new Pose(87.679, 86.903)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .setReversed()
                    .build();

            END = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.679, 86.903),
                                    new Pose(95.437, 13.541)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}
