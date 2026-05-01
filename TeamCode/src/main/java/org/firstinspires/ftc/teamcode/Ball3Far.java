package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "3 Ball", group = "Autonomous")
@Configurable
public class Ball3Far extends OpMode {

    private Follower follower;
    private Timer stateTimer;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo blocker;
    private TurretController turretController;

    private enum State {
        INITIAL_SPINUP,  // Wait for motors to reach speed
        SHOOT_PRELOADS,  // Open blocker and run intake
        MOVE_TO_POSITION, // Drive to the 28.7, 8.3 point
        DONE
    }

    private State currentState;

    // Poses from your specific coordinates
    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
    private final Pose endPose   = new Pose(28.700, 8.300, Math.toRadians(90));

    @Override
    public void init() {
        stateTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        shooter1 = hardwareMap.get(DcMotorEx.class, "RS");
        shooter2 = hardwareMap.get(DcMotorEx.class, "LS");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        turretController = new TurretController(hardwareMap, "Turret");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);

        blocker.setPosition(1.0); // Closed
        currentState = State.INITIAL_SPINUP;
    }

    @Override
    public void start() {
        stateTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        // Always aim the turret at the goal based on current position
        turretController.aimAtGoalWithPrediction(follower.getPose(), new Pose(0,0,0));

        switch (currentState) {
            case INITIAL_SPINUP:
                // Start shooters immediately
                shooter1.setVelocity(1550);
                shooter2.setVelocity(1550);

                // Give 0.8 seconds to spin up and for turret to lock on
                if (stateTimer.getElapsedTimeSeconds() > 0.8) {
                    stateTimer.resetTimer();
                    currentState = State.SHOOT_PRELOADS;
                }
                break;

            case SHOOT_PRELOADS:
                blocker.setPosition(0.3); // Open
                intake.setPower(1.0);     // Push balls out

                // Wait 1.2 seconds to ensure all preloads are gone
                if (stateTimer.getElapsedTimeSeconds() > 1.2) {
                    blocker.setPosition(1.0); // Close
                    intake.setPower(0);

                    // START MOVE AFTER SHOOTING
                    follower.followPath(follower.pathBuilder()
                            .addPath(new BezierLine(startPose, endPose))
                            .setConstantHeadingInterpolation(Math.toRadians(90))
                            .build());

                    currentState = State.MOVE_TO_POSITION;
                }
                break;

            case MOVE_TO_POSITION:
                if (!follower.isBusy()) {
                    currentState = State.DONE;
                }
                break;

            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                break;
        }

        telemetry.addData("State", currentState);
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }
}