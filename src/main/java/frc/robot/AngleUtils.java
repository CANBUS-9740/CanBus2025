package frc.robot;

public class AngleUtils {

    public static double translateAngle(double angleDegrees) {
        angleDegrees %= 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        return angleDegrees;
    }
}
