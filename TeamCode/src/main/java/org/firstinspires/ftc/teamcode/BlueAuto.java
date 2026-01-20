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
@Autonomous(name = "Big Blue Booti")
public class BlueAuto extends OpMode {

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
                new Pose(26.959, 127.625, Math.toRadians(135))
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
                    follower.followPath(paths.PickEnd);
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
       PATH DEFINITIONS
       ========================= */
    public static class Paths {

        public PathChain Shooting1;
        public PathChain PickStart1;
        public PathChain PickEnd1;
        public PathChain Shooting2;
        public PathChain PickStart2;
        public PathChain PickEnd;
        public PathChain Shooting3;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(26.959, 127.625),
                            new Pose(56.208, 87.056)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(135), Math.toRadians(128)
            ).build();

            PickStart1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.208, 87.056),
                            new Pose(46.813, 83.658)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(128), Math.toRadians(180)
            ).build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.813, 83.658),
                                    new Pose(19.577, 83.602)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(19.577, 83.602),
                            new Pose(56.737, 86.721)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(128)
            ).build();

            PickStart2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.737, 86.721),
                            new Pose(48.809, 59.741)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(128), Math.toRadians(180)
            ).build();

            PickEnd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.809, 59.741),
                                    new Pose(24.240, 59.622)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(24.240, 59.622),
                            new Pose(56.538, 86.957)
                    )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180), Math.toRadians(128)
            ).build();
        }
    }
}
