package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * TurretController with Auto-Homing, Settling Pause, and Anti-Slip Slew Rate Limiting.
 * Wrap-around hysteresis and power nerfing removed for direct boundary tracking.
 */
public class TurretController {

    private DcMotorEx turretMotor;
    private DigitalChannel limitSwitch;

    private int encoderOffset = 0;
    private boolean wasLimitTriggered = false;

    // ================= STATE MACHINE =================
    public enum TurretState { HOMING, PAUSED, TRACKING }
    public TurretState currentState = TurretState.HOMING;

    private static final double HOMING_POWER = -0.55; // Moves right to find the switch
    private static final long PAUSE_DURATION_MS = 104; // 1 second pause to prevent gear slip
    private long pauseStartTime = 0;

    // ================= GEAR PROTECTION (SLEW RATE) =================
    private static final double POWER_ACCEL_LIMIT = 5.0;
    private double currentAppliedPower = 0.0;

    // ================= MOTOR + GEAR =================
    private static final double GEAR_RATIO = 58.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // ================= LIMITS & OFFSETS =================
    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;

    private static final double HOMING_ANGLE_DEGREES = -12.17; // <-- TUNE THIS

    // ================= GOAL LOCATION =================
    private static final double GOAL_X = 5.3;
    private static final double GOAL_Y = 135.4;

    // ================= TURRET PIVOT OFFSET =================
    private static final double TURRET_OFFSET_FORWARD = 0.0;
    private static final double TURRET_OFFSET_STRAFE = 0.0;

    // ================= FINE TRIM =================
    public double ANGLE_OFFSET = 0;

    // ================= PREDICTIVE AIMING =================
    private static final double XY_SCALAR = 0.4; // Set to 0 to stop predictive jittering
    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;

    // ================= PID CONTROL (SOFT TUNING FOR PLASTIC GEARS) =================
    private static final double KP = 0.035;
    private static final double KD = 0.004;
    private static final double KF = 0.08;
    private static final double MAX_POWER = 0.85;
    private static final double DEADBAND = 1.5;

    private double targetAngle = 0.0;
    private double previousAngle = 0.0;
    private long lastTime = 0;

    // ================= CONSTRUCTOR =================
    public TurretController(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        lastTime = System.currentTimeMillis();
        currentState = TurretState.HOMING;
    }

    // ================= TELEOP AUTO-RESUME =================
    public void setSavedTicks(int savedTicks) {
        this.encoderOffset = savedTicks;
        this.currentState = TurretState.TRACKING;
        this.targetAngle = getCurrentAngle();
    }

    // ================= GETTERS =================
    public double getCurrentAngle() {
        return (turretMotor.getCurrentPosition() + encoderOffset) / COUNTS_PER_DEGREE;
    }

    public int getRawTicks() {
        return turretMotor.getCurrentPosition() + encoderOffset;
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;
        encoderOffset = 0;
        currentAppliedPower = 0.0;
        turretMotor.setPower(0);
        wasLimitTriggered = false;
        currentState = TurretState.TRACKING;
    }

    // ================= DRIFT CORRECTION =================
    private void checkHomingDrift() {
        boolean isTriggered = !limitSwitch.getState();

        if (isTriggered && !wasLimitTriggered) {
            int rawTicks = turretMotor.getCurrentPosition();
            encoderOffset = (int) (HOMING_ANGLE_DEGREES * COUNTS_PER_DEGREE) - rawTicks;
        }
        wasLimitTriggered = isTriggered;
    }

    // ================= TURRET WORLD POSITION =================
    private double[] getTurretWorldPosition(Pose robotPose) {
        double heading = robotPose.getHeading();

        double worldOffsetX = TURRET_OFFSET_FORWARD * Math.cos(heading)
                - TURRET_OFFSET_STRAFE * Math.sin(heading);
        double worldOffsetY = TURRET_OFFSET_FORWARD * Math.sin(heading)
                + TURRET_OFFSET_STRAFE * Math.cos(heading);

        return new double[]{
                robotPose.getX() + worldOffsetX,
                robotPose.getY() + worldOffsetY
        };
    }

    // ================= CALCULATION LOGIC =================
    public double getDistanceToGoal(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);
        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double calculateTurretAngle(Pose currentPose) {
        double[] turretWorld = getTurretWorldPosition(currentPose);

        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        double absTargetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = Math.toDegrees(currentPose.getHeading());
        double relativeAngle = absTargetAngle - robotHeading;

        // Standard Math Wrap to [-180, 180]
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        double finalTurretAngle = relativeAngle + ANGLE_OFFSET;

        // Let the pure math govern the target, relying purely on clip to enforce physical limits.
        return Range.clip(finalTurretAngle, MIN_ANGLE, MAX_ANGLE);
    }

    public double calculateTurretAngleWithPrediction(Pose currentPose, Pose velocity) {
        double speed = Math.sqrt(velocity.getX() * velocity.getX() + velocity.getY() * velocity.getY());

        if (speed < MIN_VELOCITY_FOR_PREDICTION) {
            return calculateTurretAngle(currentPose);
        }

        double predictedX = currentPose.getX() + (velocity.getX() * XY_SCALAR);
        double predictedY = currentPose.getY() + (velocity.getY() * XY_SCALAR);
        Pose predictedPose = new Pose(predictedX, predictedY, currentPose.getHeading());

        return calculateTurretAngle(predictedPose);
    }

    // ================= PID LOOP =================
    public void setTargetAngle(double angle) {
        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void aimAtGoalWithPrediction(Pose currentPose, Pose velocity) {
        double target = calculateTurretAngleWithPrediction(currentPose, velocity);
        setTargetAngle(target);
        update();
    }

    public void update() {
        if (currentState == TurretState.HOMING) {
            if (limitSwitch.getState() == true) {
                turretMotor.setPower(HOMING_POWER);
            } else {
                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                encoderOffset = (int) (HOMING_ANGLE_DEGREES * COUNTS_PER_DEGREE);
                previousAngle = HOMING_ANGLE_DEGREES;
                currentAppliedPower = 0.0;

                pauseStartTime = System.currentTimeMillis();
                currentState = TurretState.PAUSED;
            }
            return;
        }

        if (currentState == TurretState.PAUSED) {
            turretMotor.setPower(0);
            if (System.currentTimeMillis() - pauseStartTime >= PAUSE_DURATION_MS) {
                currentState = TurretState.TRACKING;
            }
            return;
        }

        // Apply active drift correction during tracking
        checkHomingDrift();

        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        if (dt <= 0.001) dt = 0.001;

        // --- DEADBAND (STOPPING LOGIC) ---
        if (Math.abs(error) < DEADBAND) {
            turretMotor.setPower(0);
            currentAppliedPower = 0.0;
            previousAngle = currentAngle;
            return;
        }

        // 4. PID + Feedforward Math
        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;

        // FADE OUT FIX: Only apply the violent feedforward bump if we are far away from the target
        double f = 0.0;
        if (Math.abs(error) > 3.0) {
            f = Math.signum(error) * KF;
        }

        double targetPower = p + d + f;

        // Safely clip raw target power before applying slew rate limiter
        targetPower = Range.clip(targetPower, -MAX_POWER, MAX_POWER);

        // =========================================================
        // 6. GEAR PROTECTION: ASYMMETRIC SLEW RATE LIMITER
        // =========================================================
        double maxPowerChange = POWER_ACCEL_LIMIT * dt;

        if (Math.abs(targetPower) < Math.abs(currentAppliedPower) || Math.signum(targetPower) != Math.signum(currentAppliedPower)) {
            currentAppliedPower = targetPower;
        }
        else {
            if (targetPower > currentAppliedPower + maxPowerChange) {
                currentAppliedPower += maxPowerChange;
            } else if (targetPower < currentAppliedPower - maxPowerChange) {
                currentAppliedPower -= maxPowerChange;
            } else {
                currentAppliedPower = targetPower;
            }
        }

        turretMotor.setPower(currentAppliedPower);
        previousAngle = currentAngle;
    }
}