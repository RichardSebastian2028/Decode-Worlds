package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Turret Homing Test", group = "Test")
public class TurretHomingTest extends LinearOpMode {

    private DcMotorEx turretMotor;
    private DigitalChannel limitSwitch;

    // Turret Math from your main class
    private static final double GEAR_RATIO = 189.0 / 80.0;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    // HOMING SPEED (Change to -0.15 if the turret moves left instead of right)
    private static final double HOMING_POWER = -0.15;

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Status: Ready to Home");
        telemetry.addLine("Press START to begin homing sequence.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // PHASE 1: HOMING SEQUENCE
            // ==========================================
            // limitSwitch.getState() is TRUE when no magnet is present.
            // It turns FALSE when the magnet triggers it (Blue Light ON).
            while (opModeIsActive() && limitSwitch.getState() == true) {
                turretMotor.setPower(HOMING_POWER);

                telemetry.addLine("Status: HOMING...");
                telemetry.addData("Switch State", limitSwitch.getState() + " (Looking for False)");
                telemetry.addData("Current Raw Ticks", turretMotor.getCurrentPosition());
                telemetry.update();
            }

            // ==========================================
            // PHASE 2: ZEROING
            // ==========================================
            // Magnet detected! Stop the motor and reset the encoder to 0.
            turretMotor.setPower(0);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // ==========================================
            // PHASE 3: MANUAL TEST MODE
            // ==========================================
            // Now you can drive it manually to verify it counts correctly from 0
            while (opModeIsActive()) {

                // Drive turret with left stick X
                double manualPower = gamepad1.left_stick_x * 0.4; // 40% max speed for safety
                turretMotor.setPower(manualPower);

                // Math for display
                int currentTicks = turretMotor.getCurrentPosition();
                double currentDegrees = currentTicks / COUNTS_PER_DEGREE;
                boolean isMagnetDetected = !limitSwitch.getState();

                telemetry.addLine("Status: HOMING COMPLETE - Manual Control Mode");
                telemetry.addLine("Use Gamepad 1 Left Stick to move turret.");
                telemetry.addLine("--------------------------------");
                telemetry.addData("Magnet Detected (Blue Light)", isMagnetDetected);
                telemetry.addData("Encoder Ticks", currentTicks);
                telemetry.addData("Calculated Degrees", "%.2f deg", currentDegrees);
                telemetry.update();
            }
        }
    }
}