package frc.robot.util.led;

public class AnimationEasing {
    /** @deprecated Work in progress */
    @Deprecated
    public static double easeIn(double t, double b, double c, double d) { // TODO: verify implementation
        return c * (t /= d) * t + b;
    }

    /** @deprecated Work in progress */
    @Deprecated
    public static double easeOut(double t, double b, double c, double d) { // TODO: verify implementation
        return -c * (t /= d) * (t - 2) + b;
    }

    /** @deprecated Work in progress */
    @Deprecated
    public static double easeInOut(double t, double b, double c, double d) { // TODO: verify implementation
        if ((t /= d / 2) < 1) return c / 2 * t * t + b;
        return -c / 2 * ((--t) * (t - 2) - 1) + b;
    }

    public static double sine(double input, double period, double phaseAngle) {
        return (1 - Math.cos(2 * Math.PI * (input / period - phaseAngle))) / 2;
    }
}
