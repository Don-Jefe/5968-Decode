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
@Autonomous(name = "Back Blue")
public class BackBlue extends OpMode {

    double maxp = 0.91;

    /* =========================
       STATE MACHINE
       ========================= */
    private enum AutoState {
        PRE_DIDDY,

        SHOOT_1,
        TO_PICK_1_START,
        PICK_1,

        PreJack1,
        SHOOT_2,
        TO_PICK_2_START,
        PICK_2,

        PreJack2,
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
                new Pose(58.00, 10.00, Math.toRadians(110))
        );

        follower.update();

        paths = new Paths(follower);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {

        stateTimer.resetTimer();
        drivetrain.NewSetFlywheelRPM(-3200, 18, 0.0, 0, 14);

        setShootingState();
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

        state = AutoState.SHOOT_1;

    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        drivetrain.NewSetFlywheelRPM(-3200, 18, 0.0, 0, 14);

        switch (state) {

            case PRE_DIDDY:

                drivetrain.NewSetFlywheelRPM(-3200, 18, 0.0, 0, 14);

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.8) {

                    setPickupState();

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

                    state = AutoState.SHOOT_1;

                    stateTimer.resetTimer();

                    drivetrain.NewSetFlywheelRPM(-3200, 18, 0.0, 0, 14);
                }

                break;

            case SHOOT_1:

                if (stateTimer.getElapsedTimeSeconds() >= 4) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.PickStart1);

                    state = AutoState.TO_PICK_1_START;
                }

                break;

            case TO_PICK_1_START:

                if (!follower.isBusy()) {

                    setPickupState();

                    follower.setMaxPower(.8);

                    follower.followPath(paths.PickEnd1);

                    state = AutoState.PICK_1;
                }

                break;

            case PICK_1:

                if (!follower.isBusy()) {

                    follower.setMaxPower(maxp);

                    setShootingState();

                    follower.followPath(paths.Shooting2);

                    state = AutoState.PreJack1;

                    stateTimer.resetTimer();
                }

                break;

            case PreJack1:

                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                if (!follower.isBusy()) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

                    state = AutoState.SHOOT_2;

                    stateTimer.resetTimer();
                }

                break;

            case SHOOT_2:

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 4) {

                    setPickupState();

                    follower.followPath(paths.pickStart2);

                    state = AutoState.TO_PICK_2_START;
                }

                break;

            case TO_PICK_2_START:

                if (!follower.isBusy()) {

                    setPickupState();

                    follower.setMaxPower(.32);

                    follower.followPath(paths.pickEnd2);

                    state = AutoState.PICK_2;
                }

                break;

            case PICK_2:

                if (!follower.isBusy()) {

                    follower.setMaxPower(maxp);

                    setShootingState();

                    follower.followPath(paths.Shooting3);

                    state = AutoState.PreJack2;

                    stateTimer.resetTimer();
                }

                break;

            case PreJack2:

                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                if (!follower.isBusy()) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

                    state = AutoState.SHOOT_3;

                    stateTimer.resetTimer();
                }


                break;

            case SHOOT_3:

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2) {

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

        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(0.8);
    }

    private void setPickupState() {

        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

        drivetrain.setIntakePower(-15);
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
        public PathChain pickStart2;
        public PathChain pickEnd2;
        public PathChain Shooting3;

        public Paths(Follower follower) {
            Shooting1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(58.000, 10.000),
                                    new Pose(41.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            PickStart1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(41.000, 36.000),
                                    new Pose(10.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PickEnd1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(24.000, 36.000),
                                    new Pose(58.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Shooting2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(58.000, 10.000),
                                    new Pose(30.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            pickStart2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(30.000, 10.000),
                                    new Pose(10.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickEnd2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 10.000),
                                    new Pose(10.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            Shooting3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.000, 10.000),
                                    new Pose(58.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();
        }
    }
}


