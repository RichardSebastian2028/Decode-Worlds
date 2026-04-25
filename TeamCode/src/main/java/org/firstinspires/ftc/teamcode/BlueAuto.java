package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto - Debug Drive")
public class BlueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;

    public enum PathState {
        START_DRIVE,
        MONITOR_DRIVE,
        DONE
    }

    private PathState pathState;

    private final Pose startPose = new Pose(24.6, 126.7, Math.toRadians(135));
    private final Pose pose1End  = new Pose(55.8, 89.9,  Math.toRadians(180));
    private PathChain path1;

    @Override
    public void init() {
        pathTimer = new Timer();

        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build path
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1End))
                .setConstantHeadingInterpolation(pose1End.getHeading())
                .build();

        pathState = PathState.START_DRIVE;
        telemetry.addLine("Initialized - Ready to Move");
    }

    @Override
    public void loop() {
        // CRITICAL: This must run every loop for Pedro to calculate motor powers
        follower.update();

        switch (pathState) {
            case START_DRIVE:
                // Start the path immediately
                follower.followPath(path1, true);
                pathState = PathState.MONITOR_DRIVE;
                break;

            case MONITOR_DRIVE:
                // Check if we are busy. If not busy, we have arrived.
                if (!follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;

            case DONE:
                telemetry.addLine("Path Finished!");
                break;
        }

        // --- DEBUG TELEMETRY ---
        telemetry.addData("State", pathState);
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("Current X", follower.getPose().getX());
        telemetry.addData("Current Y", follower.getPose().getY());
        telemetry.update();
    }
}