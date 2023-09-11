package org.firstinspires.ftc.teamcode.util;

public class TeamUtils {
    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
