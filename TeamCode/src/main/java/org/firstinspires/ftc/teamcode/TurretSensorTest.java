package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Test: Turret Limit Switch", group="Test")
public class TurretSensorTest extends LinearOpMode {

    private DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {
        // Change "limit" to match your configuration name in the Control Hub
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");

        // Set the switch to input mode
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Wait for Start - Move magnet near sensor to test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read the state: true usually means NOT triggered, false means triggered
            boolean isTriggered = !limitSwitch.getState();

            telemetry.addData("Sensor Raw State", limitSwitch.getState());
            telemetry.addData("Is Magnet Detected?", isTriggered ? "YES! (FALSE)" : "NO (TRUE)");
            telemetry.addLine("\nNote: Most REV Magnetic switches return FALSE when triggered.");
            telemetry.update();
        }
    }
}