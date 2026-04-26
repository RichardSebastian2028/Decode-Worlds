package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Turret Calibration Test", group="Test")
public class TurretCalibrationOpmode extends OpMode {

    private DcMotorEx turretMotor;
    private DigitalChannel limitSwitch;

    // Constants copied directly from your TurretController
    private static final double GEAR_RATIO = 189.0 / 80.0;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    private double latchedMagnetAngle = 0.0;
    private boolean hasFoundMagnet = false;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        // Resetting the encoder here makes wherever the turret is sitting exactly 0.0
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("--- INSTRUCTIONS ---");
        telemetry.addLine("1. DO NOT PRESS START YET.");
        telemetry.addLine("2. Physically move the turret to perfectly STRAIGHT FORWARD.");
        telemetry.addLine("3. Now press Start.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Slow manual control (20% power max for safety)
        double power = gamepad1.left_stick_x * 0.20;
        turretMotor.setPower(power);

        // Calculate current degrees from the "Straight Forward" starting point
        double currentAngle = turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;

        // Active low sensor check
        boolean isMagnetTriggered = !limitSwitch.getState();

        // Latch the angle the absolute first time we see the magnet
        if (isMagnetTriggered && !hasFoundMagnet) {
            latchedMagnetAngle = currentAngle;
            hasFoundMagnet = true;
        }

        telemetry.addData("Live Angle (Degrees)", "%.2f", currentAngle);
        telemetry.addData("Magnet Sensor", isMagnetTriggered ? "TRIGGERED" : "Not Triggered");

        if (hasFoundMagnet) {
            telemetry.addLine("\n=== CALIBRATION RESULT ===");
            telemetry.addData("Degrees to Magnet", "%.2f", latchedMagnetAngle);
            telemetry.addLine("Note: If this reads a negative number (e.g., -8.5),");
            telemetry.addLine("your FORWARD_OFFSET in TurretController should be positive (+8.5).");
        }

        telemetry.update();
    }
}