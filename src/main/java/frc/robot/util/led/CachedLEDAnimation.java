package frc.robot.util.led;
import java.util.function.Function;

// TODO: Implement CachedLEDAnimation

/**
 * @deprecated Unimplemented
 */
@Deprecated
public class CachedLEDAnimation extends LEDFrameAnimation {
    public CachedLEDAnimation(int length, Function<Integer, LobstahLEDBuffer> frameGenerator) {
        super(new LobstahLEDBuffer[length]);

        for (int i = 0; i < length; i++) {
            frames[i] = frameGenerator.apply(i);
        }
    }
}
