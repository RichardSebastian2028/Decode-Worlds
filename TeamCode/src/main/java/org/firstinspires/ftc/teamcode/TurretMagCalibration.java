package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Calibration: Find Mag Angle", group="Calibration")
public class TurretMagCalibration extends LinearOpMode {

    private DcMotor turretMotor;
    private DigitalChannel limitSwitch;

    // Must match your TurretController math
    private static final double GEAR_RATIO = 58 / 20;
    private static final double TICKS_PER_MOTOR_REV = 384.5;
    private static final double COUNTS_PER_DEGREE = (TICKS_PER_MOTOR_REV * GEAR_RATIO) / 360.0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "Turret");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("1. Point turret straight forward manually.");
        telemetry.addLine("2. Press Start.");
        telemetry.addLine("3. Rotate turret until magnet is found.");
        telemetry.update();

        waitForStart();

        double triggerAngle = 0;
        boolean foundTrigger = false;

        while (opModeIsActive()) {
            // Read raw sensor (False = Detected)
            boolean isTriggered = !limitSwitch.getState();

            // Calculate current angle based on 0 being where you started
            double currentAngle = turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE;

            // Manual control to move it slowly
            turretMotor.setPower(-gamepad1.left_stick_x * 0.2);

            if (isTriggered && !foundTrigger) {
                // This is the leading edge!
                triggerAngle = currentAngle;
                foundTrigger = true;
            } else if (!isTriggered) {
                foundTrigger = false;
            }

            telemetry.addData("STATUS", isTriggered ? "MAGNET DETECTED" : "Searching...");
            telemetry.addData("Current Angle", "%.2f", currentAngle);
            telemetry.addLine("---------------------------");
            telemetry.addData("USE THIS FOR PLACEHOLDER", "%.2f", triggerAngle);
            telemetry.addLine("---------------------------");
            telemetry.addLine("Slowly move turret with Left Stick X");
            telemetry.update();
        }
    }
}