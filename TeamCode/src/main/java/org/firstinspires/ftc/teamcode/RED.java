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

    double maxp = 0.9;

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
        DONE,
        PreJack1,

        PreJack2,

        PreJack3,
        PRE_DIDDY

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
                new Pose(117, 127.625, Math.toRadians(45))
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
                    drivetrain.NewSetFlywheelRPM(-2850, 18, 0.0, 0, 14);
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
                    setPickupState();
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
                    state = AutoState.PreJack1;
                    stateTimer.resetTimer();
                }
                break;
            case PreJack1:
                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                if (  !follower.isBusy()) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
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
                    setPickupState();
                    follower.setMaxPower(.25);
                    follower.followPath(paths.PickEnd2);
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
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 1.7) {
                    setPickupState();
                    follower.setMaxPower(maxp);
                    follower.followPath(paths.PickStart3);
                    state = AutoState.TO_PICK_3_START;
                }
                break;

            case TO_PICK_3_START:
                if (!follower.isBusy()) {
                    setPickupState();
                    follower.setMaxPower(.4);
                    follower.followPath(paths.PickEnd3);
                    state = AutoState.PICK_3;
                }
                break;

            case PICK_3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(maxp);
                    setShootingState();
                    follower.followPath(paths.Shooting4);
                    state = AutoState.PreJack3;
                    stateTimer.resetTimer();
                }
                break;
            case PreJack3:
                drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
                if (!follower.isBusy()) {
                    drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
                    state = AutoState.SHOOT_4;
                    stateTimer.resetTimer();
                }
                break;

            case SHOOT_4:
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.0) {
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

        telemetryM.debug("Auto State", state);
        telemetryM.debug("State Time", stateTimer.getElapsedTimeSeconds());
    }

    /* =========================
       MECHANISM STATES
       ========================= */
    private void setShootingState() {
        drivetrain.setIntakePower(-1.0);
        drivetrain.setFeederPower(1.0);
    }

    private void setPickupState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_TOP_POS);
        drivetrain.setIntakePower(-.75);
        drivetrain.setFeederPower(0.6);
    }

    private void setIdleState() {
        drivetrain.blocker.setPosition(Drivetrain.SERVO_BOTTOM_POS);
        drivetrain.setIntakePower(0);
        drivetrain.setFeederPower(0);
    }

    /* =========================
       PATH DEFINITIONS (NEW)
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
                                    new Pose(97.187, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.187, 80),
                                    new Pose(127.891, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.891, 83.207),
                                    new Pose(87.263, 86.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            PickStart2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.263, 86.721),
                                    new Pose(96.007, 55)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            PickEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.007, 55),
                                    new Pose(121, 55)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121, 57),
                                    new Pose(87.365, 86.983)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            PickStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.365, 86.983),
                                    new Pose(96.693, 32)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                    .build();

            PickEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.693, 32),
                                    new Pose(130.198, 32)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shooting4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130, 32),
                                    new Pose(87.679, 86.903)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                    .build();

            END = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.679, 86.903),
                                    new Pose(95.437, 13.541)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }

}
