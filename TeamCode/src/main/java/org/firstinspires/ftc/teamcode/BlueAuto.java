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
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2) {
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
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2) {
                    setPickupState();
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
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() >= 2.5) {
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
                                    new Pose(26.959, 127.625),
                                    new Pose(56.208, 87.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            PickStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.208, 87.056),
                                    new Pose(46.813, 82.658)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            PickEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.813, 82.658),
                                    new Pose(12, 82.394)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.28)

                    .build();

            Shooting2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12, 82.394),
                                    new Pose(56.737, 86.721)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            PickStart2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.737, 86.721),
                                    new Pose(47.993, 57.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.993, 57.000),
                                    new Pose(15, 57.00)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.25)

                    .build();

            Shooting3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15, 58.826),
                                    new Pose(56.635, 86.983)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            PickStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.635, 86.983),
                                    new Pose(47.307, 33.913)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            PickEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(47.307, 33.913),
                                    new Pose(4, 33.970)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0.35)

                    .build();

            Shooting4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(4, 33.970),
                                    new Pose(56.321, 86.903)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            END = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.321, 86.903),
                                    new Pose(48.563, 13.541)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }
}
