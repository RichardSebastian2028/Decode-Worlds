package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

    // --- Subsystems ---
    private Follower follower;
    private TurretController turretController;

    private DcMotor leftFront, leftRear, rightFront, rightRear, intake;
    private DcMotorEx leftShooter, rightShooter;
    private Servo blockerServo;

    // --- SHOOTER PIDF COEFFICIENTS ---
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 3.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 13.5;

    // --- SHOOTER REGRESSION CONSTANTS ---
    private final double REG_A = 0.0321632;   // quadratic
    private final double REG_B = -0.489268;   // linear
    private final double REG_C = 1236.75193;  // constant base speed

    // --- Holding Poses ---
    private final Pose resetPose  = new Pose(7, 9, Math.toRadians(90));
    private final Pose emptyGate  = new Pose(132.5, 68.5, Math.toRadians(180));

    private boolean holdingEmptyGate  = false;
    private boolean firstLoop         = true;

    // Velocity Prediction Variables
    private Pose lastPose = null;
    private long lastTime = 0;

    @Override
    public void init() {
        // --- 1. Pedro Pathing Follower (FIXED FOR NEW UPDATE) ---
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(new Pose(54.9, 7.4, Math.toRadians(90)));

        // --- 2. Turret Controller ---
        turretController = new TurretController(hardwareMap, "Turret");

        // --- 3. Drivetrain Hardware ---
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightRear = hardwareMap.get(DcMotor.class, "BR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 4. Intake & Blocker ---
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blockerServo = hardwareMap.get(Servo.class, "blocker");

        // --- 5. Shooter Setup ---
        leftShooter = hardwareMap.get(DcMotorEx.class, "LS");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RS");

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        if (firstLoop) { firstLoop = false; return; }

        if (gamepad1.triangle) follower.setPose(resetPose);

        // --- Hold Point Example ---
        if (gamepad1.square && !holdingEmptyGate) {
            follower.holdPoint(emptyGate);
            holdingEmptyGate = true;
        }
        if (!gamepad1.square && holdingEmptyGate) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            holdingEmptyGate = false;
        }

        // --- Drivetrain ---
        double driveForward = holdingEmptyGate ? 0 : gamepad1.left_stick_y;
        double driveStrafe  = holdingEmptyGate ? 0 : -gamepad1.left_stick_x;
        double driveTurn    = holdingEmptyGate ? 0 : -gamepad1.right_stick_x;
        follower.setTeleOpDrive(-driveForward, driveStrafe, driveTurn, true);

        // --- Prediction Math ---
        Pose currentPose = follower.getPose();
        Pose velocity;
        long currentTime = System.currentTimeMillis();

        if (lastPose == null || lastTime == 0) {
            velocity = new Pose(0, 0, 0);
        } else {
            double dt = (currentTime - lastTime) / 1000.0;
            if (dt > 0.005) {
                double velX = (currentPose.getX() - lastPose.getX()) / dt;
                double velY = (currentPose.getY() - lastPose.getY()) / dt;
                double diffHeading = currentPose.getHeading() - lastPose.getHeading();
                while (diffHeading > Math.PI)  diffHeading -= 2 * Math.PI;
                while (diffHeading < -Math.PI) diffHeading += 2 * Math.PI;
                velocity = new Pose(velX, velY, diffHeading / dt);
            } else {
                velocity = new Pose(0, 0, 0);
            }
        }
        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastTime = currentTime;

        // --- Turret Aiming ---
        turretController.aimAtGoalWithPrediction(currentPose, velocity);

        if (gamepad2.options) turretController.resetEncoder();
        if (gamepad2.dpad_left) turretController.ANGLE_OFFSET += 0.2;
        if (gamepad2.dpad_right) turretController.ANGLE_OFFSET -= 0.2;

        // --- Intake & Blocker (SEPARATED!) ---
        // Right Bumper = Intake | Cross/A = Outtake
        intake.setPower(gamepad2.right_bumper ? 1.0 : (gamepad2.cross ? -0.9 : 0.0));

        // Left Bumper = Open Blocker (Shoot) | Otherwise Closed
        blockerServo.setPosition(gamepad2.left_bumper ? 0.3 : 1.0);

        // --- Shooter (CONSTANT RUN) ---
        double distance = turretController.getDistanceToGoal(currentPose);
        double targetVel = (REG_A * Math.pow(distance, 2)) + (REG_B * distance) + REG_C;

        // Flywheels constantly update to the predicted velocity
        leftShooter.setVelocity(targetVel);
        rightShooter.setVelocity(targetVel);

        // --- Telemetry ---
        telemetry.addData("Distance", "%.2f", distance);
        telemetry.addData("Turret Angle", "%.2f", turretController.getCurrentAngle());
        telemetry.update();
    }
}