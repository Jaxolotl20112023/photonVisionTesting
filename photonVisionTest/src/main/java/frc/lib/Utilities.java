package frc.lib;

public class Utilities {
    public static double polynomialAccleration(double x) {
        return Math.pow(x,3) * 0.88 + Math.pow(x,2) * 0.12;
    }

    public static double convertYawReadings(double reading) {
        double processedReading = reading % 360;
        if(processedReading > 180) {
            processedReading = processedReading - 360;
        }
        return processedReading;
    }

    public static double processYaw(double yaw) {
        if(yaw < 0) {
            return 360 - (Math.abs(yaw) % 360);
        } else {
            return yaw % 360;
        }
    }
}
