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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable
public class BLUE12BALLAUTONEW extends OpMode {
    private TelemetryManager panelsTelemetry;

    // Mechanisms
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx turret;
    private DcMotor intake;
    private Servo kicker;

    public double shootVelocity = 1530;
    private static final int TURRET_TARGET_TICKS = 103;

    // Software
    public Follower follower;
    private Timer pathTimer, opModeTimer, kickTimer;

    // Shooting state (Blue18 style)
    private boolean shooting = false;
    private int ballsShot = 0;

    // States
    public enum PathState {
        DRIVE_PATH1,
        SPIN_UP1,
        SHOOT1,
        INTAKECLOSE1,
        DRIVE_PATH2,
        DRIVE_PATH3,
        INTAKE_ON_PATH3,
        DRIVE_PATH4,
        DRIVE_PATH5,
        SPIN_UP2,
        SHOOT2,
        INTAKECLOSE2,
        DRIVE_PATH6,
        DRIVE_PATH7,
        INTAKE_ON_PATH7,
        DRIVE_PATH8,
        SPIN_UP3,
        SHOOT3,
        INTAKECLOSE3,
        DRIVE_PATH9,
        DRIVE_PATH10,
        DRIVE_PATH11,
        SPIN_UP4,
        SHOOT4,
        INTAKECLOSE4,
        DRIVE_PATH12,
        DONE,
    }

    PathState pathState;

    // Poses
    private final Pose startPose = new Pose(24.6, 126.7, Math.toRadians(135));
    private final Pose pose1End  = new Pose(55.8, 89.9,  Math.toRadians(180));
    private final Pose pose2End  = new Pose(45.3, 61.6,  Math.toRadians(180));
    private final Pose pose3End  = new Pose(12.8, 61.6,  Math.toRadians(180));
    private final Pose bezierIntakeToGate = new Pose(34.2, 67.1, Math.toRadians(180));
    private final Pose pose4End  = new Pose(13.2, 71.5,  Math.toRadians(180));
    private final Pose bezierGateToShoot  = new Pose(61.4, 61.3, Math.toRadians(180));
    private final Pose pose5End  = new Pose(55.8, 89.9,  Math.toRadians(180));
    private final Pose pose6End  = new Pose(45,   83.5,  Math.toRadians(180));
    private final Pose pose7End  = new Pose(23.9, 83.5,  Math.toRadians(180));
    private final Pose pose8End  = new Pose(55.8, 89.9,  Math.toRadians(180));
    private final Pose pose9End  = new Pose(45.4, 36.5,  Math.toRadians(180));
    private final Pose pose10End = new Pose(12.9, 36.5,  Math.toRadians(180));
    private final Pose pose11End = new Pose(55.8, 89.9,  Math.toRadians(180));
    private final Pose pose12End = new Pose(57.3, 110.2, Math.toRadians(180));

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8,
            path9, path10, path11, path12;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1End))
                .setConstantHeadingInterpolation(pose1End.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1End, pose2End))
                .setLinearHeadingInterpolation(pose1End.getHeading(), pose2End.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2End, pose3End))
                .setLinearHeadingInterpolation(pose2End.getHeading(), pose3End.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(pose3End, bezierIntakeToGate, pose4End))
                .setLinearHeadingInterpolation(pose3End.getHeading(), pose4End.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(pose4End, bezierGateToShoot, pose5End))
                .setConstantHeadingInterpolation(pose5End.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5End, pose6End))
                .setLinearHeadingInterpolation(pose5End.getHeading(), pose6End.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6End, pose7End))
                .setLinearHeadingInterpolation(pose6End.getHeading(), pose7End.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(pose7End, pose8End))
                .setConstantHeadingInterpolation(pose8End.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(pose8End, pose9End))
                .setLinearHeadingInterpolation(pose8End.getHeading(), pose9End.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(pose9End, pose10End))
                .setLinearHeadingInterpolation(pose9End.getHeading(), pose10End.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(pose10End, pose11End))
                .setConstantHeadingInterpolation(pose11End.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(pose11End, pose12End))
                .setLinearHeadingInterpolation(pose11End.getHeading(), pose12End.getHeading())
                .build();
    }

    // Blue18 style: only resets timer, does NOT reset ballsShot
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_PATH1:
                follower.followPath(path1, true);
                setPathState(PathState.SPIN_UP1);
                break;

            case SPIN_UP1:
                // Wait for path to finish + spin-up time
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT1);
                }
                break;

            case SHOOT1:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        intake.setPower(1);
                        follower.followPath(path2, true);
                        setPathState(PathState.INTAKECLOSE1);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE1:
                kicker.setPosition(0.15);
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
                    follower.followPath(path5, true);
                    setPathState(PathState.DRIVE_PATH5);
                }
                break;

            case DRIVE_PATH5:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.SPIN_UP2);
                }
                break;

            case SPIN_UP2:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        intake.setPower(1);
                        follower.followPath(path6, true);
                        setPathState(PathState.INTAKECLOSE2);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE2:
                kicker.setPosition(0.15);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        intake.setPower(1);
                        setPathState(PathState.INTAKECLOSE3);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE3:
                kicker.setPosition(0.15);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    ballsShot = 0;
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (!shooting && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.setPower(1);
                    kicker.setPosition(0.25);
                    shooting = true;
                    kickTimer.resetTimer();
                } else if (shooting && kickTimer.getElapsedTimeSeconds() > 0.16) {
                    kicker.setPosition(0.31);
                    shooting = false;
                    ballsShot++;

                    if (ballsShot >= 3) {
                        ballsShot = 0;
                        intake.setPower(1);
                        follower.followPath(path12, true);
                        setPathState(PathState.INTAKECLOSE4);
                    } else {
                        pathTimer.resetTimer();
                    }
                }
                break;
            case INTAKECLOSE4:
                kicker.setPosition(0.15);
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
                kicker.setPosition(0.31);
                turret.setDirection(DcMotorSimple.Direction.REVERSE);
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setTargetPosition(-1000);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.5);
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
        shooter1 = hardwareMap.get(DcMotorEx.class, "RS");
        shooter2 = hardwareMap.get(DcMotorEx.class, "LS");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "blocker");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(TURRET_TARGET_TICKS);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.5);
        kicker.setPosition(0.15);
        buildPaths();
        pathState = PathState.DRIVE_PATH1;
        shooting = false;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        // Shooter starts immediately at full velocity (Blue18 style)
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);
    }

    @Override
    public void loop() {
        follower.update();

        // Blue18 style: velocity re-set every loop so it never drops
        shooter1.setVelocity(shootVelocity);
        shooter2.setVelocity(shootVelocity);

        statePathUpdate();

        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Balls Shot", ballsShot);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Shooter1 Velocity", shooter1.getVelocity());
        panelsTelemetry.debug("Shooter2 Velocity", shooter2.getVelocity());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
    }
}