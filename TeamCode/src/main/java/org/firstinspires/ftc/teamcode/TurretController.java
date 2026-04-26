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
    private static final long PAUSE_DURATION_MS = 1000; // 1 second pause to prevent gear slip
    private long pauseStartTime = 0;

    // ================= GEAR PROTECTION (SLEW RATE) =================
    private static final double POWER_ACCEL_LIMIT = 1.5; // Max power change per second
    private double currentAppliedPower = 0.0;

    // ================= MOTOR + GEAR =================
    private static final double GEAR_RATIO = 58 / 20;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // ================= LIMITS =================
    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;

    // ================= GOAL LOCATION =================
    private static final double GOAL_X = 5.3;
    private static final double GOAL_Y = 135.4
            ;

    // ================= TURRET PIVOT OFFSET =================
    private static final double TURRET_OFFSET_FORWARD = 0.0;
    private static final double TURRET_OFFSET_STRAFE = 0.0;

    // ================= FINE TRIM =================
    public double ANGLE_OFFSET = 0;

    // ================= PREDICTIVE AIMING =================
    private static final double XY_SCALAR = 0.0; // Set to 0 to stop predictive jittering
    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;

    // ================= PID CONTROL (SOFT TUNING FOR PLASTIC GEARS) =================
    private static final double KP = 0.035;
    private static final double KD = 0.001;
    private static final double KF = 0.06;
    private static final double MAX_POWER = 0.6; // Capped speed to protect gears
    private static final double DEADBAND = 0.5;

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
            encoderOffset = -rawTicks; // The limit switch IS exactly 0
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

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle <= -180) relativeAngle += 360;

        double finalTurretAngle = relativeAngle + ANGLE_OFFSET;
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
        // =========================================================
        // PHASE 1: HOMING SEQUENCE
        // =========================================================
        if (currentState == TurretState.HOMING) {
            if (limitSwitch.getState() == true) { // Magnet NOT detected yet
                turretMotor.setPower(HOMING_POWER);
            } else { // Magnet DETECTED!
                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoderOffset = 0;
                previousAngle = 0;
                currentAppliedPower = 0.0;

                // Start the pause timer and switch state
                pauseStartTime = System.currentTimeMillis();
                currentState = TurretState.PAUSED;
            }
            return;
        }

        // =========================================================
        // PHASE 1.5: SETTLING PAUSE
        // =========================================================
        if (currentState == TurretState.PAUSED) {
            turretMotor.setPower(0); // Ensure motor stays off

            // If 1 second (1000ms) has passed, proceed to tracking
            if (System.currentTimeMillis() - pauseStartTime >= PAUSE_DURATION_MS) {
                currentState = TurretState.TRACKING;
            }
            return;
        }

        // =========================================================
        // PHASE 2: NORMAL TRACKING (PID + SLEW RATE)
        // =========================================================

        // 1. Correct drift if we pass over the limit switch
        //checkHomingDrift();

        // 2. Calculate Angle and Wrap Error
        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        // 3. Time Delta calculation
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
        double f = Math.signum(error) * KF;

        double targetPower = p + d + f;

        // 5. Safety Limits near 180/-180
        double currentMaxPower = MAX_POWER;
        if ((currentAngle < MIN_ANGLE + 10.0 && targetPower < 0) || (currentAngle > MAX_ANGLE - 10.0 && targetPower > 0)) {
            currentMaxPower = 0.20;
        }
        targetPower = Range.clip(targetPower, -currentMaxPower, currentMaxPower);

        // =========================================================
        // 6. GEAR PROTECTION: SLEW RATE LIMITER
        // =========================================================
        double maxPowerChange = POWER_ACCEL_LIMIT * dt;

        if (targetPower > currentAppliedPower + maxPowerChange) {
            currentAppliedPower += maxPowerChange;
        } else if (targetPower < currentAppliedPower - maxPowerChange) {
            currentAppliedPower -= maxPowerChange;
        } else {
            currentAppliedPower = targetPower;
        }

        // Apply smoothed power to motor
        turretMotor.setPower(currentAppliedPower);
        previousAngle = currentAngle;
    }
}