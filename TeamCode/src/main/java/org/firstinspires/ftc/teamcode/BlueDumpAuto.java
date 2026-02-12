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
@Autonomous(name = "Blue Dump Auto")
public class BlueDumpAuto extends OpMode {

    double maxp = 0.9;

    /* =========================
       STATE MACHINE
       ========================= */
    private enum AutoState {
        PRE_DIDDY,

        SHOOT_1,
        TO_PICK_1_START,
        PICK_1,

        SHOOT_2,
        TO_PICK_2_START,
        PICK_2,

        DUMP,
        PRE_SHOOT_3,
        SHOOT_3,

        PARK,
        DONE
    }

    private AutoState state;

    /* ========================= */
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
        drivetrain.NewSetFlywheelRPM(-2850, 18, 0.0, 0, 14);
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

        follower.followPath(paths.Shooting1);

        state = AutoState.PRE_DIDDY;
        stateTimer.resetTimer();
    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        drivetrain.NewSetFlywheelRPM(-2850, 18, 0.0, 0, 14);

        switch (state) {

            case PRE_DIDDY:
                drivetrain.NewSetFlywheelRPM(-2950, 18, 0.0, 0, 14);

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.8) {
                    setPickupState();
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_1;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_1:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                    setPickupState();
                    follower.followPath(paths.PickStart1);
                    state = AutoState.TO_PICK_1_START;
                }
                break;

            case TO_PICK_1_START:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.45);
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
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 1.7) {
                    setPickupState();
                    follower.followPath(paths.PickStart2);
                    state = AutoState.TO_PICK_2_START;
                }
                break;

            case TO_PICK_2_START:
                if (!follower.isBusy()) {
                    follower.setMaxPower(.3);
                    follower.followPath(paths.PickEnd2);
                    state = AutoState.PICK_2;
                }
                break;

            case PICK_2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(maxp);
                    setPickupState();
                    follower.followPath(paths.Dump);
                    stateTimer.resetTimer();
                    state = AutoState.DUMP;
                }
                break;

            case DUMP:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 5) {
                    setShootingState();
                    follower.followPath(paths.Shooting3);
                    state = AutoState.PRE_SHOOT_3;
                    stateTimer.resetTimer();
                }
                break;

            case PRE_SHOOT_3:
                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                if (!follower.isBusy()) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_3;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_3:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 1.7) {
                    setPickupState();
                    follower.followPath(paths.Park);
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

    /* ========================= */
    private void setShootingState() {
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(1.0);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-0.75);
        drivetrain.setFeederPower(0.6);
    }

    private void setIdleState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(0);
        drivetrain.setFeederPower(0);
    }

    /* =========================
       PATHS (blue mirrored)
       ========================= */
    public static class Paths {

        public PathChain Shooting1;
        public PathChain PickStart1;
        public PathChain PickEnd1;

        public PathChain Shooting2;
        public PathChain PickStart2;
        public PathChain PickEnd2;

        public PathChain Dump;
        public PathChain Shooting3;
        public PathChain Park;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(26.959, 127.625),
                            new Pose(56.208, 87.056)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135)).build();

            PickStart1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.208, 87.056),
                            new Pose(46.813, 82.658)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            PickEnd1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(46.813, 82.658),
                            new Pose(12, 82.394)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Shooting2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(12, 82.394),
                            new Pose(56.737, 86.721)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            PickStart2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.737, 86.721),
                            new Pose(47.993, 57.000)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            PickEnd2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(47.993, 57.000),
                            new Pose(12, 57)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Dump = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(12, 57),
                            new Pose(63, 61),
                            new Pose(17, 69)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)).build();

            Shooting3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(17, 69),
                            new Pose(56.6, 86.9)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135)).build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(56.6, 86.9),
                            new Pose(48.5, 110)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(135)).build();
        }
    }
}
