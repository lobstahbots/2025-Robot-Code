package frc.robot.util.led;

// TODO: Implement LEDFrameAnimation

/**
 * @deprecated Unimplemented
 */
@Deprecated
public class LEDFrameAnimation {
    public final LobstahLEDBuffer[] frames;
    public final boolean loop = false;

    public LEDFrameAnimation(LobstahLEDBuffer... frames) {
        this.frames = frames;
    }

    public LobstahLEDBuffer getFrame(int index) {
        if (loop) return frames[Math.floorMod(index, frames.length)];
        else if (index < 0 || index >= frames.length) return null;
        else return frames[index];
    }

    public int getLength() {
        return frames.length;
    }

    public LobstahLEDBuffer getFrame(double index) { // TODO
        int i = (int) index;
        double alpha = index - i;
        LobstahLEDBuffer frameA = frames[i];
        LobstahLEDBuffer frameB = frames[i + 1];
        return LobstahLEDBuffer.layer(Math.max(frameA.length, frameB.length), frameA, frameB);
    }
}
