package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NotOpModes.CF;
import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.NotOpModes.PoseStorage;


@Configurable
@TeleOp(name = "Jeff's Final Tele-Op")
public class VibeCodeTele extends OpMode {

    /* =========================
       FIELD CONSTANTS
       ========================= */
    public static final double FIELD_SIZE = 144.0;

    public static final double BLUE_RESET_X = 24.24;
    public static final double BLUE_RESET_Y = 127.03;
    public static final double BLUE_RESET_HEADING = Math.toRadians(130);

    public static final double RED_RESET_X = 125.0;
    public static final double RED_RESET_Y = 130.0;
    public static final double RED_RESET_HEADING = Math.toRadians(37);

    public static final double RED_BACK_RESET_X= 136;
    public static final double RED_BACK_RESET_Y = 8;
    public static final double DEFAULT_RESET_HEADING = Math.toRadians(90);


    public static final double BlUE_BACK_RESET_X = 8;
    public static final double BLUE_BACK_RESET_Y = 8;

    /* =========================
       AIMBOT CONSTANTS
       ========================= */
    public static double TURN_kP = 2;
    public static double MAX_TURN_POWER = 0.40;
    public static double AIM_DELAY_SEC = 0.10;

    /* =========================
       HARDWARE
       ========================= */
    private Follower follower;
    private Drivetrain drivetrain;
    private TelemetryManager telemetryM;

    /* =========================
       TARGETING
       ========================= */
    private double targetX;
    private double targetY;
    private boolean isBlueAlliance = true;

    /* =========================
       AIMBOT STATE
       ========================= */
    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean wasAimbotActive = false;

    /* =========================
       NEW FLYWHEEL RAMP
       ========================= */
    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double lastLoopTime = 0;

    public static double RPM_ACCEL = 6000;
    public static double RPM_DECEL = 9000;

    /* =========================
       HELPER METHODS
       ========================= */
    private double getTurnToTarget(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double desiredHeading = Math.atan2(dy, dx);
        double error = normalizeAngle(desiredHeading - pose.getHeading());
        telemetryM.debug("Actual Heading", desiredHeading);
        return clamp(error * TURN_kP, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void updateAllianceTarget() {
        if (isBlueAlliance) {
            targetX = 8;
            targetY = FIELD_SIZE - 8;
        } else {
            targetX = FIELD_SIZE - 8;
            targetY = FIELD_SIZE - 8;
        }
    }

    /* =========================
       DISTANCE → RPM
       ========================= */
    private double getDistanceToBasket(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        return Math.hypot(dx, dy);
    }

    private double getFlywheelRPMForDistance(double distance) {
        if (distance < 140) {
            return 0.0701569 * distance * distance + -7.65244 * distance + 2808.8812;
        } else return 3100;


    }

    private double getActualFlywheelRPM() {
        double ticksPerSecond = drivetrain.flywheel.getVelocity();
        return (ticksPerSecond / 28.0) * 60.0;
    }

    private void updateFlywheelRamp(double dt) {
        if (rampedRPM < targetRPM) {
            rampedRPM += RPM_ACCEL * dt;
            if (rampedRPM > targetRPM) rampedRPM = targetRPM;
        } else {
            rampedRPM -= RPM_DECEL * dt;
            if (rampedRPM < targetRPM) rampedRPM = targetRPM;
        }
    }

    /* =========================
       INIT
       ========================= */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        updateAllianceTarget();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastLoopTime = getRuntime();
    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();
        if (isBlueAlliance) {
            gamepad1.setLedColor(0, 0, 255,1000);
        } else {
            gamepad1.setLedColor(255, 0, 0,1000);
        }

        Pose pose = follower.getPose();

        /* =========================
           ALLIANCE TOGGLE
           ========================= */
        if (gamepad1.psWasPressed()) {
            isBlueAlliance = !isBlueAlliance;
            updateAllianceTarget();
        }

        /* =========================
           POSE RESET
           ========================= */
        if (gamepad1.dpadUpWasPressed()) {
            follower.setPose(isBlueAlliance
                    ? new Pose(BLUE_RESET_X, BLUE_RESET_Y, BLUE_RESET_HEADING)
                    : new Pose(RED_RESET_X, RED_RESET_Y, RED_RESET_HEADING));
        } else if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(isBlueAlliance
                    ? new Pose(BlUE_BACK_RESET_X, BLUE_BACK_RESET_Y, DEFAULT_RESET_HEADING)
                    : new Pose(RED_BACK_RESET_X, RED_BACK_RESET_Y, DEFAULT_RESET_HEADING));
        }


        /* =========================
           AIMBOT + DRIVE
           ========================= */
        boolean aimbotActive = gamepad1.right_trigger > 0.7;

        if (aimbotActive && !wasAimbotActive) {
            aimTimer.reset();
        }
        wasAimbotActive = aimbotActive;

        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = aimbotActive ? getTurnToTarget(pose) : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);

        /* =========================
           TARGET RPM
           ========================= */
        double distance = getDistanceToBasket(pose);
        targetRPM = getFlywheelRPMForDistance(distance);

        /* =========================
           RAMP
           ========================= */
        double now = getRuntime();
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        updateFlywheelRamp(dt);

        drivetrain.NewSetFlywheelRPM(-rampedRPM, CF.PPP, 0, 0, CF.FFF);

        /* =========================
           MECHANISMS (UNCHANGED)
           ========================= */
        if (aimbotActive && aimTimer.seconds() >= AIM_DELAY_SEC) {
            drivetrain.setIntakePower(-1);
            drivetrain.setFeederPower(1);
        } else if (!(gamepad1.left_trigger > 0.5)) {
            drivetrain.setIntakePower(0);
            drivetrain.setFeederPower(0);
        }

        drivetrain.updateBlocker(gamepad1);
        drivetrain.updateIntake(gamepad1.left_trigger);
        drivetrain.updateFeeder(gamepad1);

        /* =========================
           TELEMETRY
           ========================= */
        telemetryM.debug("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetryM.debug("Pose", pose);
        telemetryM.debug("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetryM.debug("Distance to basket (in)", distance);
        telemetryM.debug("Target Flywheel RPM", targetRPM);
        telemetryM.debug("Ramped RPM", rampedRPM);
        telemetryM.debug("Actual Flywheel RPM", getActualFlywheelRPM());
        telemetryM.debug("Aimbot Active", aimbotActive);
        telemetryM.debug("Aim Timer (s)", aimTimer.seconds());
        telemetryM.debug("Actual Heading", pose.getHeading());
    }
}
