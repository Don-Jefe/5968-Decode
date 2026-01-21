package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NotOpModes.Drivetrain;

@Configurable
@TeleOp(name = "Jeff's Aimbot Tele")
public class DecodeTele extends OpMode {

    /* =========================
       FIELD CONSTANTS
       ========================= */
    public static final double FIELD_SIZE = 144.0;

    public static final double BLUE_RESET_X = 24.24;
    public static final double BLUE_RESET_Y = 127.03;
    public static final double BLUE_RESET_HEADING = Math.toRadians(130);

    public static final double RED_RESET_X = 120.0;
    public static final double RED_RESET_Y = 130.0;
    public static final double RED_RESET_HEADING = Math.toRadians(37);

    /* =========================
       AIMBOT CONSTANTS
       ========================= */
    public static double TURN_kP = 2.0;
    public static double MAX_TURN_POWER = 1.0;
    public static double AIM_DELAY_SEC = 0.20;

    /* =========================
       FLYWHEEL RPM CONSTANTS
       ========================= */
    public static double MIN_SHOT_DISTANCE = 55;   // in inches
    public static double MAX_SHOT_DISTANCE = 135.0;  // in inches
    public static double MIN_FLYWHEEL_RPM = 2500;
    public static double MAX_FLYWHEEL_RPM = 3075;

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
       HELPER METHODS
       ========================= */
    private double getTurnToTarget(Pose pose) {
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double desiredHeading = Math.atan2(dy, dx);
        double error = normalizeAngle(desiredHeading - pose.getHeading());
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
            targetY = FIELD_SIZE-8;
        } else {
            targetX = FIELD_SIZE-8;
            targetY = FIELD_SIZE-8;
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
        distance = clamp(distance, MIN_SHOT_DISTANCE, MAX_SHOT_DISTANCE);
        double t = (distance - MIN_SHOT_DISTANCE) / (MAX_SHOT_DISTANCE - MIN_SHOT_DISTANCE);
        return MIN_FLYWHEEL_RPM + t * (MAX_FLYWHEEL_RPM - MIN_FLYWHEEL_RPM);
    }
    private double getActualFlywheelRPM() {
        // encoder velocity is ticks per second
        double ticksPerSecond = drivetrain.flywheel.getVelocity();
        return (ticksPerSecond / 24) * 60.0;
    }


    /* =========================
       INIT
       ========================= */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.PI / 2));
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        updateAllianceTarget();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /* =========================
       LOOP
       ========================= */
    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

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
        if (gamepad1.dpadDownWasPressed()) {
            follower.setPose(isBlueAlliance
                    ? new Pose(BLUE_RESET_X, BLUE_RESET_Y, BLUE_RESET_HEADING)
                    : new Pose(RED_RESET_X, RED_RESET_Y, RED_RESET_HEADING));
        }

        /* =========================
           AIMBOT + DRIVE
           ========================= */
        boolean aimbotActive = gamepad1.right_trigger > 0.5;

        // detect rising edge
        if (aimbotActive && !wasAimbotActive) {
            aimTimer.reset();
        }
        wasAimbotActive = aimbotActive;

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn = aimbotActive ? getTurnToTarget(pose) : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, turn, true);

        /* =========================
           CALCULATE DISTANCE AND RPM
           ========================= */
        double distance = getDistanceToBasket(pose);
        double targetRPM = getFlywheelRPMForDistance(distance);

        if (aimbotActive) {
            drivetrain.setFlywheelRPM(-targetRPM);
        }

        /* =========================
           MECHANISMS (INTAKE / FEEDER)
           ========================= */
        if (aimbotActive && aimTimer.seconds() >= AIM_DELAY_SEC) {
            drivetrain.setIntakePower(-1);
            drivetrain.setFeederPower(1);
        } else if (!(gamepad1.left_trigger > 0.5)) {
            drivetrain.setIntakePower(0);
            drivetrain.setFeederPower(0);
        }

        drivetrain.updateBlocker(gamepad1);
        drivetrain.updateFlywheel(gamepad1);
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
        telemetryM.debug("Actual Flywheel RPM", getActualFlywheelRPM());
        telemetryM.debug("Aimbot Active", aimbotActive);
        telemetryM.debug("Aim Timer (s)", aimTimer.seconds());

    }
}
