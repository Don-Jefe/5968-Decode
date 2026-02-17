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
import org.firstinspires.ftc.teamcode.NotOpModes.PoseStorage;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Blue 3/Dump/6")
public class BlueDumpAuto extends OpMode {

    double maxp = 0.72;

    /* ================= STATE MACHINE ================= */
    private enum AutoState {
        TO_SHOOT_1, SHOOT_1,
        TO_PICK_1_START,
        TO_PICK_1_END,
        TO_MIDPOINT,
        TO_DUMP_START,
        TO_DUMP_END,
        TO_SHOOT_2, SHOOT_2,
        TO_PICK_2_START,
        TO_PICK_2_END,
        DODGE,
        TO_SHOOT_3, SHOOT_3,
        TO_PARK,
        DONE
    }

    private AutoState state;

    /* ================= HARDWARE ================= */
    private Follower follower;
    private Drivetrain drivetrain;
    private Paths paths;
    private TelemetryManager telemetryM;
    private final Timer stateTimer = new Timer();

    /* ================= INIT ================= */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(26.959, 127.625, Math.toRadians(137)));
        follower.update();

        paths = new Paths(follower);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /* ================= START ================= */
    @Override
    public void start() {
        drivetrain.NewSetFlywheelRPM(-2850, 18, 0, 0, 14);
        follower.followPath(paths.Shooting1);
        state = AutoState.TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {
        follower.setMaxPower(maxp);
        follower.update();
        telemetryM.update();

        drivetrain.NewSetFlywheelRPM(-2850, 18, 0, 0, 14.5);

        switch (state) {

            case TO_SHOOT_1:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.67) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    setShootingState();
                    state = AutoState.SHOOT_1;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_1:
                if (stateTimer.getElapsedTimeSeconds() >= 2.3) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                    setPickupState();
                    follower.followPath(paths.PickStart1);
                    state = AutoState.TO_PICK_1_START;
                }
                break;

            case TO_PICK_1_START:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickEnd1);
                    state = AutoState.TO_PICK_1_END;
                }
                break;

            case TO_PICK_1_END:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Midpoint);
                    state = AutoState.TO_MIDPOINT;
                }
                break;

            case TO_MIDPOINT:
                if (!follower.isBusy()) {
                    follower.followPath(paths.DumpStart);
                    state = AutoState.TO_DUMP_START;
                }
                break;

            case TO_DUMP_START:
                if (!follower.isBusy()) {
                    follower.followPath(paths.DumpEnd);
                    state = AutoState.TO_DUMP_END;
                    stateTimer.resetTimer();
                }
                break;

            case TO_DUMP_END:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 3.8) {
                    setShootingState();
                    follower.followPath(paths.Shooting2);
                    state = AutoState.TO_SHOOT_2;
                }
                break;

            case TO_SHOOT_2:
                if (!follower.isBusy()) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_2;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_2:
                if (stateTimer.getElapsedTimeSeconds() >= 2) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                    setPickupState();
                    follower.followPath(paths.PickStart2);
                    state = AutoState.TO_PICK_2_START;
                }
                break;

            case TO_PICK_2_START:
                if (!follower.isBusy()) {
                    follower.followPath(paths.PickEnd2);
                    state = AutoState.TO_PICK_2_END;
                }
                break;

            case TO_PICK_2_END:
                if (!follower.isBusy()) {
                    setShootingState();
                    follower.followPath(paths.Dodge);
                    state = AutoState.DODGE;
                }
                break;

            case DODGE:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shooting3);
                    state = AutoState.TO_SHOOT_3;
                }
                break;

            case TO_SHOOT_3:
                if (!follower.isBusy()) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_3;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_3:
                if (stateTimer.getElapsedTimeSeconds() >= 2) {
                    setPickupState();
                    follower.followPath(paths.Park);
                    state = AutoState.TO_PARK;
                }
                break;

            case TO_PARK:
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                PoseStorage.currentPose = follower.getPose();
                drivetrain.setIntakePower(0);
                drivetrain.setFeederPower(0);
                break;
        }

        telemetryM.debug("Auto State", state);
        telemetryM.debug("State Time", stateTimer.getElapsedTimeSeconds());
    }

    /* ================= MECHANISMS ================= */
    private void setShootingState() {
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(0.80);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-.75);
        drivetrain.setFeederPower(0.6);
    }

    /* ================= PATHS ================= */
    public static class Paths {

        public PathChain Shooting1, PickStart1, PickEnd1, Midpoint;
        public PathChain DumpStart, DumpEnd, Shooting2;
        public PathChain PickStart2, PickEnd2, Dodge, Shooting3, Park;

        public Paths(Follower follower) {

            Shooting1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(26.959, 127.625), new Pose(56.208, 87.056))
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135)).build();

            PickStart1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(56.208, 87.056), new Pose(46.813, 80))
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            PickEnd1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(46.813, 80), new Pose(12.109, 80))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Midpoint = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(12.109, 80), new Pose(28, 72))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            DumpStart = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(28, 72), new Pose(50.410, 70))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0)).build();

            DumpEnd = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(50.410, 70), new Pose(25.205, 64))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Shooting2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(25.205, 64), new Pose(56.737, 86.721))
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135)).build();

            PickStart2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(56.737, 86.721), new Pose(60.424, 56.357))
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)).build();

            PickEnd2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(60.424, 56.357), new Pose(6.311, 56.492))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

            Dodge = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(6.311, 56.492), new Pose(25.311, 53.492))
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)).build();

            Shooting3 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(25.311, 53.492), new Pose(57.033, 87.232))
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135)).build();

            Park = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(57.033, 87.232), new Pose(54.984, 117.443))
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135)).build();
        }
    }
}
