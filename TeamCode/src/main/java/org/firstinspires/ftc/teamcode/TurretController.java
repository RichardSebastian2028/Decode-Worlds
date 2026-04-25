package org.firstinspires.ftc.teamcode;



import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;



/**

 * TurretController with Magnetic Homing and Anti-Slip logic.

 * Designed to aim at a goal while preventing mechanical drift.

 */

public class TurretController {



    private DcMotorEx turretMotor;

    private DigitalChannel limitSwitch;



    private int encoderOffset = 0;

    private boolean wasLimitTriggered = false;



// ================= MOTOR + GEAR =================

// Input : Output = 80 : 189

    private static final double GEAR_RATIO = 189.0 / 80.0;

    private static final double TICKS_PER_MOTOR_REV = 384.5;

    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;



// ================= LIMITS =================

    private static final double MIN_ANGLE = -180.0;

    private static final double MAX_ANGLE = 180.0;



// ================= MAGNETIC SENSOR HOMING =================

// Based on your calibration: -5.55 degrees

    private static final double MAGNETIC_SWITCH_ANGLE = -5.55;



// ================= GOAL LOCATION =================

    private static final double GOAL_X = 2.7;

    private static final double GOAL_Y = 140.2;



// ================= TURRET PIVOT OFFSET =================

    private static final double TURRET_OFFSET_FORWARD = 0;

    private static final double TURRET_OFFSET_STRAFE = 0.0;



// ================= FINE TRIM =================

    public double ANGLE_OFFSET = 0.0;



// ================= PREDICTIVE AIMING =================

    private static final double XY_SCALAR = 0.4;

    private static final double MIN_VELOCITY_FOR_PREDICTION = 2.0;



// ================= PID CONTROL (STIFF TUNING) =================

    private static final double KP = 0.035; // Increased for "stiffer" response

    private static final double KD = 0.0008; // Added a tiny touch of D to dampen the stiff P

    private static final double MAX_POWER = 0.6; // Increased slightly now that it's stable

    private static final double DEADBAND = 0.5; // Tight target window



    private double targetAngle = 0.0;

    private double previousAngle = 0.0;

    private long lastTime = 0;



// ================= CONSTRUCTOR =================

    public TurretController(HardwareMap hardwareMap, String motorName) {

        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);



        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



// Ensure this matches your physical assembly (FORWARD or REVERSE)

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);



// Initialize Limit Switch (Name: "turretLimit" in Config)

        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);



        lastTime = System.currentTimeMillis();

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

        turretMotor.setPower(0);

        wasLimitTriggered = false;

    }



// ================= HOMING LOGIC =================

    private void checkHoming() {

        boolean isTriggered = !limitSwitch.getState();



// Edge Detection: Re-calibrate the moment the magnet is detected

        if (isTriggered && !wasLimitTriggered) {

            int rawTicks = turretMotor.getCurrentPosition();

            double expectedTicks = MAGNETIC_SWITCH_ANGLE * COUNTS_PER_DEGREE;



// Re-calculate offset to kill any drift

            encoderOffset = (int) Math.round(expectedTicks) - rawTicks;

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

// 1. Correct drift using magnetic sensor

        checkHoming();



// 2. PID Math

        double currentAngle = getCurrentAngle();

        double error = targetAngle - currentAngle;



        long now = System.currentTimeMillis();

        double dt = (now - lastTime) / 1000.0;

        lastTime = now;



        if (dt <= 0.001) dt = 0.001;



// --- ANTI-SLIP HOLDING FORCE ---

        if (Math.abs(error) < DEADBAND){

// Apply 8% power to lock gears and prevent sliding

            turretMotor.setPower(0.08 * Math.signum(error));

            previousAngle = currentAngle;

            return;

        }



        double p = KP * error;

        double derivative = (currentAngle - previousAngle) / dt;

        double d = -KD * derivative;



        double power = p + d;



// Static friction breakout bump

        if (Math.abs(power) < 0.12 && Math.abs(power) > 0.01){

            power = Math.signum(power) * 0.12;

        }



// Safety Limits near 180/-180

        double currentMaxPower = MAX_POWER;

        if ((currentAngle < MIN_ANGLE + 10.0 && power < 0) || (currentAngle > MAX_ANGLE - 10.0 && power > 0)) {

            currentMaxPower = 0.20;

        }



        power = Range.clip(power, -currentMaxPower, currentMaxPower);

        turretMotor.setPower(power);

        previousAngle = currentAngle;

    } }