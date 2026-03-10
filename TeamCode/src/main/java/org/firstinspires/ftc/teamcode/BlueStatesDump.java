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
@Autonomous(name = "15 Blue Ball's Dump Front")
public class BlueStatesDump extends OpMode {

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
        follower.setStartingPose(new Pose(26.959, 127.625, Math.toRadians(135)));
        follower.update();

        paths = new Paths(follower);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {

        drivetrain.setDualFlywheelRPM(-2900, 18, 0, 0, 14);

        follower.followPath(paths.Shooting1);

        state = AutoState.TO_SHOOT_1;
        stateTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.setMaxPower(maxp);
        follower.update();

        drivetrain.setBothFlywheelRPM(-2920, 18, 0, 0, 14.5);

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

                if (stateTimer.getElapsedTimeSeconds() > 1.8) {

                    drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);

                    setPickupState();

                    follower.followPath(paths.Dump);

                    state = AutoState.TO_DUMP;
                }

                break;

            case TO_DUMP:

                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 4.3) {

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
                                    new Pose(26.959, 127.625),

                                    new Pose(56.208, 87.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                    .build();

            PickStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.208, 87.056),

                                    new Pose(44.771, 61.190)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.771, 61.190),

                                    new Pose(15.697, 60.930)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.697, 60.930),
                                    new Pose(51.921, 61.123),
                                    new Pose(56.153, 86.925)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Dump = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.153, 86.925),
                                    new Pose(50.596, 28.045),
                                    new Pose(25.178, 95.811),
                                    new Pose(6.349, 51.042)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(120))

                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.349, 51.042),

                                    new Pose(55.519, 86.919)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(145))

                    .build();

            PickStart2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.519, 86.919),

                                    new Pose(47.889, 85.149)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .build();

            PickEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.889, 85.149),

                                    new Pose(17.557, 84.664)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shooting4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.557, 84.664),

                                    new Pose(56.119, 87.140)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            PickStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.119, 87.140),

                                    new Pose(41.681, 37.128)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.681, 37.128),

                                    new Pose(13.468, 36.826)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shooting5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.468, 36.826),

                                    new Pose(58.621276595744675, 106.91914893617023)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.621276595744675, 106.91914893617023),

                                    new Pose(61.562, 110.038)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(145))

                    .build();
        }
    }

}