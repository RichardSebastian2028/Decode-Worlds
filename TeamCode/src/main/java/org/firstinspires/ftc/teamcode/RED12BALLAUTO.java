package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class RED12BALLAUTO extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo blocker;

    // External Turret Controller
    private TurretController turretController;

    public double shootVelocity = 1500;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    // Manual Velocity Calculation
    private Pose lastPose = new Pose(0,0,0);
    private long lastUpdateTime = 0;
    private Pose currentVelocity = new Pose(0,0,0);

    // Shooting state
    private boolean shooting = false;
    private int ballsShot = 0;

    public enum PathState {
        DRIVE_PATH1, SPIN_UP1, SHOOT1, INTAKECLOSE1,
        DRIVE_PATH2, DRIVE_PATH3, INTAKE_ON_PATH3,
        DRIVE_PATH4, DRIVE_PATH5, SPIN_UP2, SHOOT2, INTAKECLOSE2,
        DRIVE_PATH6, DRIVE_PATH7, INTAKE_ON_PATH7,
        DRIVE_PATH8, SPIN_UP3, SHOOT3, INTAKECLOSE3,
        DRIVE_PATH9, DRIVE_PATH10, DRIVE_PATH11,
        SPIN_UP4, SHOOT4, INTAKECLOSE4, DRIVE_PATH12, DONE
    }

    PathState pathState;

    // Red Side Poses
    private final Pose startPose = new Pose(119.4, 126.7, Math.toRadians(45));
    private final Pose pose1End  = new Pose(88.2, 89.9, Math.toRadians(0));
    private final Pose pose2End  = new Pose(99.6, 60.0, Math.toRadians(0));
    private final Pose pose3End  = new Pose(134.0, 60.0, Math.toRadians(0));
    private final Pose bezierIntakeToGate = new Pose(109.8, 67.1, Math.toRadians(0));
    private final Pose pose4End  = new Pose(130.8, 71.5, Math.toRadians(0));
    private final Pose bezierGateToShoot  = new Pose(82.6, 61.3, Math.toRadians(0));
    private final Pose pose5End  = new Pose(88.2, 89.9, Math.toRadians(0));
    private final Pose pose6End  = new Pose(101.6, 85.4, Math.toRadians(0));
    private final Pose pose7End  = new Pose(133.4, 85.4, Math.toRadians(0));
    private final Pose pose8End  = new Pose(88.2, 89.9, Math.toRadians(0));
    private final Pose pose9End  = new Pose(98.6, 36.5, Math.toRadians(0));
    private final Pose pose10End = new Pose(131.1, 36.5, Math.toRadians(0));
    private final Pose pose11End = new Pose(88.2, 89.9, Math.toRadians(0));
    private final Pose pose12End = new Pose(86.7, 110.2, Math.toRadians(0));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8,
            path9, path10, path11, path12;

    public void buildPaths() {
        path1 = follower.pathBuilder().addPath(new BezierLine(startPose, pose1End)).setConstantHeadingInterpolation(pose1End.getHeading()).build();
        path2 = follower.pathBuilder().addPath(new BezierLine(pose1End, pose2End)).setLinearHeadingInterpolation(pose1End.getHeading(), pose2End.getHeading()).build();
        path3 = follower.pathBuilder().addPath(new BezierLine(pose2End, pose3End)).setLinearHeadingInterpolation(pose2End.getHeading(), pose3End.getHeading()).build();
        path4 = follower.pathBuilder().addPath(new BezierCurve(pose3End, bezierIntakeToGate, pose4End)).setLinearHeadingInterpolation(pose3End.getHeading(), pose4End.getHeading()).build();
        path5 = follower.pathBuilder().addPath(new BezierCurve(pose4End, bezierGateToShoot, pose5End)).setConstantHeadingInterpolation(pose5End.getHeading()).build();
        path6 = follower.pathBuilder().addPath(new BezierLine(pose5End, pose6End)).setLinearHeadingInterpolation(pose5End.getHeading(), pose6End.getHeading()).build();
        path7 = follower.pathBuilder().addPath(new BezierLine(pose6End, pose7End)).setLinearHeadingInterpolation(pose6End.getHeading(), pose7End.getHeading()).build();
        path8 = follower.pathBuilder().addPath(new BezierLine(pose7End, pose8End)).setConstantHeadingInterpolation(pose8End.getHeading()).build();
        path9 = follower.pathBuilder().addPath(new BezierLine(pose8End, pose9End)).setLinearHeadingInterpolation(pose8End.getHeading(), pose9End.getHeading()).build();
        path10 = follower.pathBuilder().addPath(new BezierLine(pose9End, pose10End)).setLinearHeadingInterpolation(pose9End.getHeading(), pose10End.getHeading()).build();
        path11 = follower.pathBuilder().addPath(new BezierLine(pose10End, pose11End)).setConstantHeadingInterpolation(pose11End.getHeading()).build();
        path12 = follower.pathBuilder().addPath(new BezierLine(pose11End, pose12End)).setLinearHeadingInterpolation(pose11End.getHeading(), pose12End.getHeading()).build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        follower.setMaxPower(0.8);
        switch (pathState) {
            case DRIVE_PATH1:
                follower.followPath(path1, true);
                setPathState(PathState.SPIN_UP1);
                break;

            case SPIN_UP1:
                blocker.setPosition(0.3);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                intake.setPower(1.0);
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path2, true);
                    setPathState(PathState.INTAKECLOSE1); // Correct
                }
                break;

            case INTAKECLOSE1:
                blocker.setPosition(1.0);
                setPathState(PathState.DRIVE_PATH2);
                break;

            case DRIVE_PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(PathState.DRIVE_PATH3);
                }
                break;

            case DRIVE_PATH3:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    setPathState(PathState.INTAKE_ON_PATH3);
                }
                break;

            case INTAKE_ON_PATH3:
                follower.followPath(path4, true);
                setPathState(PathState.DRIVE_PATH4);
                break;

            case DRIVE_PATH4:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(path5, true);
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            case DRIVE_PATH5:
                if (!follower.isBusy()) {
                    setPathState(PathState.SPIN_UP2);
                }
                break;

            case SPIN_UP2:
                blocker.setPosition(0.3);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                intake.setPower(1.0);
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path6, true);
                    setPathState(PathState.INTAKECLOSE2); // CHANGED: Was INTAKECLOSE1
                }
                break;

            case INTAKECLOSE2:
                blocker.setPosition(1.0);
                setPathState(PathState.DRIVE_PATH6);
                break;

            case DRIVE_PATH6:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(PathState.DRIVE_PATH7);
                }
                break;

            case DRIVE_PATH7:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    setPathState(PathState.INTAKE_ON_PATH7);
                }
                break;

            case INTAKE_ON_PATH7:
                follower.followPath(path8, true);
                setPathState(PathState.DRIVE_PATH8);
                break;

            case DRIVE_PATH8:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP3);
                }
                break;

            case SPIN_UP3:
                blocker.setPosition(0.3);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                intake.setPower(1.0);
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path9, true);
                    setPathState(PathState.INTAKECLOSE3); // CHANGED: Was INTAKECLOSE1
                }
                break;

            case INTAKECLOSE3:
                blocker.setPosition(1.0);
                setPathState(PathState.DRIVE_PATH9);
                break;

            case DRIVE_PATH9:
                if (!follower.isBusy()) {
                    follower.followPath(path10, true);
                    setPathState(PathState.DRIVE_PATH10);
                }
                break;

            case DRIVE_PATH10:
                if (!follower.isBusy()) {
                    follower.followPath(path11, true);
                    setPathState(PathState.DRIVE_PATH11);
                }
                break;

            case DRIVE_PATH11:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP4);
                }
                break;

            case SPIN_UP4:
                blocker.setPosition(0.3);
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                intake.setPower(1.0);
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(path12, true);
                    setPathState(PathState.INTAKECLOSE4); // CHANGED: Was INTAKECLOSE1
                }
                break;

            case INTAKECLOSE4:
                blocker.setPosition(1.0);
                setPathState(PathState.DRIVE_PATH12);
                break;

            case DRIVE_PATH12:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                shooter1.setVelocity(0);
                shooter2.setVelocity(0);
                intake.setPower(0);
                blocker.setPosition(0.3);
                telemetry.addLine("Done all Paths");
                break;
        }
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        kickTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        lastPose = startPose;

        shooter1 = hardwareMap.get(DcMotorEx.class, "RS");
        shooter2 = hardwareMap.get(DcMotorEx.class, "LS");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        blocker = hardwareMap.get(Servo.class, "blocker");

        turretController = new TurretController(hardwareMap, "Turret");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(90, 0, 0, 16);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        blocker.setPosition(1.0);
        buildPaths();
        pathState = PathState.DRIVE_PATH1;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        follower.update();

        // Manual Velocity Calculation
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;
        Pose currentPose = follower.getPose();

        if (dt > 0) {
            double vx = (currentPose.getX() - lastPose.getX()) / dt;
            double vy = (currentPose.getY() - lastPose.getY()) / dt;
            double vHeading = (currentPose.getHeading() - lastPose.getHeading()) / dt;
            currentVelocity = new Pose(vx, vy, vHeading);
        }

        lastPose = currentPose;
        lastUpdateTime = currentTime;

        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        statePathUpdate();

        // Feed tracking and prediction to the external class
        turretController.aimAtGoalWithPrediction(currentPose, currentVelocity);

        // Telemetry
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(currentPose.getHeading()));
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("VX", currentVelocity.getX());
        panelsTelemetry.debug("VY", currentVelocity.getY());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
    }
}