package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Shooter Velocity Tuner", group="Testing")
public class ShooterTuner extends OpMode {

    private Follower follower;
    private TurretController turretController;
    private DcMotor intake;
    private DcMotorEx leftShooter, rightShooter;
    private Servo blockerServo;

    // --- Tuning Variables ---
    private double targetVelocity = 1300; // Starting point
    private double turretManualPower = 0;

    // Debounce for dpad
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Start Teleop Drive so we can move the robot to different distances
        follower.startTeleopDrive();

        turretController = new TurretController(hardwareMap, "Turret");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        blockerServo = hardwareMap.get(Servo.class, "blocker");
        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Standard PIDF from your MainTeleOp
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(90, 3, 0, 15);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);

        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        follower.update();

        // 1. Move the Robot (Standard Drive)
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        // 2. Manual Turret (Gamepad 2 Left Stick)
        // This lets you aim manually to ensure you are centered on the goal

        // 3. Adjust Velocity (Dpad Up/Down)
        if (gamepad2.dpad_up && !dpadUpPressed) {
            targetVelocity += 50;
            dpadUpPressed = true;
        } else if (!gamepad2.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad2.dpad_down && !dpadDownPressed) {
            targetVelocity -= 50;
            dpadDownPressed = true;
        } else if (!gamepad2.dpad_down) {
            dpadDownPressed = false;
        }

        // 4. Run Flywheels Constantly
        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);

        // 5. Shooting Controls
        intake.setPower(gamepad2.right_bumper ? 1.0 : (gamepad2.cross ? -0.9 : 0.0));
        blockerServo.setPosition(gamepad2.left_bumper ? 0.3 : 1.0);

        // --- Data Collection Telemetry ---
        Pose currentPose = follower.getPose();
        double distance = turretController.getDistanceToGoal(currentPose);

        telemetry.addLine("--- REGRESSION TUNING DATA ---");
        telemetry.addData("DISTANCE (X)", "%.2f inches", distance);
        telemetry.addData("TARGET VELOCITY (Y)", "%.0f ticks/sec", targetVelocity);
        telemetry.addLine("------------------------------");
        telemetry.addData("Current Velocity LS", leftShooter.getVelocity());
        telemetry.addData("Current Velocity RS", rightShooter.getVelocity());
        telemetry.addData("Robot Pose", currentPose.toString());
        telemetry.update();
    }
}