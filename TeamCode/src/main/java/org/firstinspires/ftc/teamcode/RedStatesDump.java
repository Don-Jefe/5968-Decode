package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.NotOpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "15 Red Ball + Dump Front")
public class RedStatesDump extends OpMode {

    double maxp = 1;

    private Follower follower;
    private Drivetrain drivetrain;
    private Paths paths;
    private TelemetryManager telemetryM;

    private final Timer stateTimer = new Timer();

    private enum AutoState {

        TO_SHOOT_1, SHOOT_1,

        TO_PICK_START_1,
        TO_PICK_END_1,

        TO_SHOOT_2, SHOOT_2,

        TO_DUMP,

        TO_SHOOT_3, SHOOT_3,

        TO_PICK_START_2,
        TO_PICK_END_2,

        TO_SHOOT_4, SHOOT_4,

        TO_PICK_START_3,
        TO_PICK_END_3,

        TO_SHOOT_5, SHOOT_5,

        TO_PARK,

        DONE
    }

    private AutoState state;

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

        drivetrain.NewSetFlywheelRPM(-2850, 18, 0, 0, 14);

        follower.followPath(paths.Shooting1);

        state = AutoState.TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.setMaxPower(maxp);
        follower.update();

        drivetrain.NewSetFlywheelRPM(-2850, 18, 0, 0, 14.5);

        switch (state) {

            case TO_SHOOT_1:

                if (!follower.isBusy()) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    setShootingState();

                    state = AutoState.SHOOT_1;
                    stateTimer.resetTimer();
                }

                break;

            case SHOOT_1:

                if (stateTimer.getElapsedTimeSeconds() > 1.6) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.PickStart1);

                    state = AutoState.TO_PICK_START_1;
                }

                break;

            case TO_PICK_START_1:

                if (!follower.isBusy()) {

                    follower.followPath(paths.PickEnd1);

                    state = AutoState.TO_PICK_END_1;
                }

                break;

            case TO_PICK_END_1:

                if (!follower.isBusy()) {

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

                if (stateTimer.getElapsedTimeSeconds() > 2) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.Dump);

                    state = AutoState.TO_DUMP;
                }

                break;

            case TO_DUMP:

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 3.4) {

                    setShootingState();

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

                if (stateTimer.getElapsedTimeSeconds() > 1.6) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.PickStart2);

                    state = AutoState.TO_PICK_START_2;
                }

                break;

            case TO_PICK_START_2:

                if (!follower.isBusy()) {

                    follower.followPath(paths.PickEnd2);

                    state = AutoState.TO_PICK_END_2;
                }

                break;

            case TO_PICK_END_2:

                if (!follower.isBusy()) {

                    setShootingState();

                    follower.followPath(paths.Shooting4);

                    state = AutoState.TO_SHOOT_4;
                }

                break;

            case TO_SHOOT_4:

                if (!follower.isBusy()) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

                    state = AutoState.SHOOT_4;
                    stateTimer.resetTimer();
                }

                break;

            case SHOOT_4:

                if (stateTimer.getElapsedTimeSeconds() > 1.6) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.PickStart3);

                    state = AutoState.TO_PICK_START_3;
                }

                break;

            case TO_PICK_START_3:

                if (!follower.isBusy()) {

                    follower.followPath(paths.PickEnd3);

                    state = AutoState.TO_PICK_END_3;
                }

                break;

            case TO_PICK_END_3:

                if (!follower.isBusy()) {

                    setShootingState();

                    follower.followPath(paths.Shooting5);

                    state = AutoState.TO_SHOOT_5;
                }

                break;

            case TO_SHOOT_5:

                if (!follower.isBusy()) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);

                    state = AutoState.SHOOT_5;
                    stateTimer.resetTimer();
                }

                break;

            case SHOOT_5:

                if (stateTimer.getElapsedTimeSeconds() > 2) {

                    follower.followPath(paths.Path13);

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

        telemetryM.debug("State", state);
        telemetryM.update();
    }

    private void setShootingState() {

        drivetrain.setIntakePower(-1);
        drivetrain.setFeederPower(.95);
    }

    private void setPickupState() {

        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-1);
        drivetrain.setFeederPower(.8);
    }



        public static class Paths {
            public PathChain Shooting1;
            public PathChain PickStart1;
            public PathChain PickEnd1;
            public PathChain Shooting2;
            public PathChain Dump;
            public PathChain Shooting3;
            public PathChain PickStart2;
            public PathChain PickEnd2;
            public PathChain Shooting4;
            public PathChain PickStart3;
            public PathChain PickEnd3;
            public PathChain Shooting5;
            public PathChain Path13;

            public Paths(Follower follower) {
                Shooting1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(117.041, 127.625),

                                        new Pose(87.792, 87.056)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(49))

                        .build();

                PickStart1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(87.792, 87.056),

                                        new Pose(96.165, 62.212)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))

                        .build();

                PickEnd1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(96.165, 62.212),

                                        new Pose(128.303, 61.951)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Shooting2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(128.303, 61.951),
                                        new Pose(92.079, 61.123),
                                        new Pose(87.847, 86.925)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))

                        .build();

                Dump = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(87.847, 86.925),
                                        new Pose(100.306, 35.399),
                                        new Pose(128.906, 92.543),
                                        new Pose(134.587, 48.289)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(60))

                        .build();

                Shooting3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(134.587, 48.289),

                                        new Pose(87.481, 86.919)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(49))

                        .build();

                PickStart2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(87.481, 86.919),

                                        new Pose(96.111, 86.170)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))

                        .build();

                PickEnd2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(96.111, 86.170),

                                        new Pose(125.217, 85.481)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Shooting4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(125.217, 85.481),

                                        new Pose(87.881, 87.140)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))

                        .build();

                PickStart3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(87.881, 87.140),

                                        new Pose(100.277, 38.353)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))

                        .build();

                PickEnd3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(100.277, 38.353),

                                        new Pose(128.489, 38.460)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Shooting5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(128.489, 38.460),

                                        new Pose(89.055, 112.247)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                        .build();

                Path13 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(89.055, 112.247),

                                        new Pose(97.757, 124.949)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))

                        .build();

        }

    }
}