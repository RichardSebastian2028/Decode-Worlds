package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * Static storage for robot pose and turret state that persists between OpModes.
 * Allows TeleOp to know where Autonomous actually ended so it doesn't snap or re-home.
 */
public class PedroPose {

    // ================= TURRET STATE =================
    private static Integer lastKnownTurretTicks = null;

    public static void saveTurretTicks(int ticks) {
        lastKnownTurretTicks = ticks;
    }

    public static Integer getTurretTicks() {
        return lastKnownTurretTicks;
    }

    // ================= DRIVETRAIN POSE =================
    private static Pose lastKnownPose = null;
    private static boolean poseSetByAuto = false;

    public static void saveCurrentPose(Pose pose) {
        lastKnownPose = pose;
        poseSetByAuto = true;
    }

    public static Pose getLastKnownPose() {
        return lastKnownPose;
    }

    public static boolean hasPoseFromAuto() {
        return poseSetByAuto && lastKnownPose != null;
    }

    // ================= RESET / UTILITIES =================
    public static void clearSavedPose() {
        lastKnownPose = null;
        poseSetByAuto = false;
        lastKnownTurretTicks = null;
    }

    public static Pose getDefaultTeleOpStartPose() {
        // Default fallback if Auto wasn't run before TeleOp.
        // Set to your standard Blue TeleOp starting location.
        return new Pose(54.9, 7.4, Math.toRadians(90));
    }

    public static Pose getTeleOpStartPose() {
        if (hasPoseFromAuto()) {
            return lastKnownPose;
        } else {
            return getDefaultTeleOpStartPose();
        }
    }
}