package org.firstinspires.ftc.teamcode.lib;


import org.firstinspires.ftc.teamcode.lib.math.InterpolatingDouble;
import org.firstinspires.ftc.teamcode.lib.math.InterpolatingTreeMap;

public class Constants {
    public static class ShootingParams{
        public static double[][] kRPMValues = {
                {13.85,2800},
                {0.0,3000},
                {-7.5,3100}, // SONRADAN EKLENDİ DÜZELTME
                {-10.5,3450}, // SONRADAN EKLENDİ DÜZELTME
                {-12.55,3500},
                {-16.5,4100},
                {-17.0,4300}
        };
        public static double[][] kHoodValues = {
                {13.85,0.0},
                {0.0,10.0},
                {-12.55,20},
                {-16.5,32.0},
                {-17.0,32.0}
        };
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRPMMap = new InterpolatingTreeMap<>();
        static {
            for (double[] pair : kRPMValues) {
                kRPMMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }

            for (double[] pair : kHoodValues) {
                kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
            }
        }
    }

}
