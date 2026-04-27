package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * TurretControllerRed with Auto-Homing, Settling Pause, and Anti-Slip Slew Rate Limiting.
 */
public class TurretControllerRed {

    private DcMotorEx turretMotor;
    private DigitalChannel limitSwitch;

    private int encoderOffset = 0;
    private boolean wasLimitTriggered = false;

    // ================= STATE MACHINE =================
    public enum TurretState { HOMING, PAUSED, TRACKING }
    public TurretState currentState = TurretState.HOMING;

    private static final double HOMING_POWER = -0.55;
    private static final long PAUSE_DURATION_MS = 150;
    private long pauseStartTime = 0;

    // ================= GEAR PROTECTION (SLEW RATE) =================
    private static final double POWER_ACCEL_LIMIT = 1.5;
    private double currentAppliedPower = 0.0;

    // ================= MOTOR + GEAR =================
    private static final double GEAR_RATIO = 58.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // ================= LIMITS & OFFSETS =================
    private static final double MIN_ANGLE = -180.0;
    private static final double MAX_ANGLE = 180.0;

    private static final double HOMING_ANGLE_DEGREES = -12.17; // <-- TUNE THIS

    // ================= RED GOAL LOCATION =================
    private static final double GOAL_X = 138.7; // 144 - 5.3
    private static final double GOAL_Y = 135.4; // Remains the same in Pedro Pathing

    // ================= TURRET PIVOT OFFSET =================
    private static final double TURRET_OFFSET_FORWARD = 0.0;
    private static final double TURRET_OFFSET_STRAFE = 0.0;

    // ================= FINE TRIM =================
    public double ANGLE_OFFSET = 0;

    // ================= PREDICTIVE AIMING =================
    private static final double XY_SCALAR = 0.4;
    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;

    // ================= PID CONTROL =================
    private static final double KP = 0.035;
    private static final double KD = 0.001;
    private static final double KF = 0.12;
    private static final double MAX_POWER = 0.6;
    private static final double DEADBAND = 0.2;

    private double targetAngle = 0.0;
    private double previousAngle = 0.0;
    private long lastTime = 0;

    // ================= CONSTRUCTOR =================
    public TurretControllerRed(HardwareMap hardwareMap, String motorName) {
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
    public double getCurrentAngleRed() {
        return (turretMotor.getCurrentPosition() + encoderOffset) / COUNTS_PER_DEGREE;
    }

    public int getRawTicksRed() {
        return turretMotor.getCurrentPosition() + encoderOffset;
    }

    public void resetEncoderRed() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngle = 0;

        encoderOffset = 0;
        currentAppliedPower = 0.0;
        turretMotor.setPower(0);
        wasLimitTriggered = false;
        currentState = TurretState.TRACKING;
    }

    // ================= TURRET WORLD POSITION =================
    private double[] getTurretWorldPositionRed(Pose robotPose) {
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
    public double getDistanceToRedGoal(Pose currentPose) {
        double[] turretWorld = getTurretWorldPositionRed(currentPose);
        double dx = GOAL_X - turretWorld[0];
        double dy = GOAL_Y - turretWorld[1];
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double calculateRedTurretAngle(Pose currentPose) {
        double[] turretWorld = getTurretWorldPositionRed(currentPose);

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

    public double calculateRedTurretAngleWithPrediction(Pose currentPose, Pose velocity) {
        double speed = Math.sqrt(velocity.getX() * velocity.getX() + velocity.getY() * velocity.getY());

        if (speed < MIN_VELOCITY_FOR_PREDICTION) {
            return calculateRedTurretAngle(currentPose);
        }

        double predictedX = currentPose.getX() + (velocity.getX() * XY_SCALAR);
        double predictedY = currentPose.getY() + (velocity.getY() * XY_SCALAR);
        Pose predictedPose = new Pose(predictedX, predictedY, currentPose.getHeading());

        return calculateRedTurretAngle(predictedPose);
    }

    // ================= PID LOOP =================
    public void setTargetAngleRed(double angle) {
        this.targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void aimAtRedGoalWithPrediction(Pose currentPose, Pose velocity) {
        double target = calculateRedTurretAngleWithPrediction(currentPose, velocity);
        setTargetAngleRed(target);
        updateRed();
    }

    public void updateRed() {
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

        double currentAngle = getCurrentAngleRed();
        double error = targetAngle - currentAngle;

        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        if (dt <= 0.001) dt = 0.001;

        if (Math.abs(error) < DEADBAND) {
            turretMotor.setPower(0);
            currentAppliedPower = 0.0;
            previousAngle = currentAngle;
            return;
        }

        double p = KP * error;
        double derivative = (currentAngle - previousAngle) / dt;
        double d = -KD * derivative;
        double f = Math.signum(error) * KF;

        double targetPower = p + d + f;

        double currentMaxPower = MAX_POWER;
        if ((currentAngle < MIN_ANGLE + 10.0 && targetPower < 0) || (currentAngle > MAX_ANGLE - 10.0 && targetPower > 0)) {
            currentMaxPower = 0.20;
        }
        targetPower = Range.clip(targetPower, -currentMaxPower, currentMaxPower);

        double maxPowerChange = POWER_ACCEL_LIMIT * dt;

        if (Math.abs(targetPower) < Math.abs(currentAppliedPower) || Math.signum(targetPower) != Math.signum(currentAppliedPower)) {
            currentAppliedPower = targetPower;
        } else {
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