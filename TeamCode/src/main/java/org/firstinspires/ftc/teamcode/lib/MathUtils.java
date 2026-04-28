package org.firstinspires.ftc.teamcode.lib;

public class MathUtils {
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static boolean epsilonEquals(double val1, double val2) {
        return Math.abs(val1 - val2) < 1e-6;
    }
    public static double getRadRotDist(double start, double end){
        double diff = (end - start + Math.PI) % (2 * Math.PI) - Math.PI;
        return diff < -Math.PI ? (diff + (Math.PI * 2)) : diff;
    }

    public static double getRotDist(double start, double end){
        return MathUtils.getRadRotDist(start, end);
    }

    public static double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private static double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}