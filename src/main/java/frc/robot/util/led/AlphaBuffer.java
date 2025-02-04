package frc.robot.util.led;

import java.util.Arrays;

public class AlphaBuffer {
    // A wrapper for an array of doubles
    public final double[] buffer;

    public AlphaBuffer(double[] alpha) {
        this.buffer = alpha;
    }

    public AlphaBuffer(int length) {
        this(new double[length]);
    }

    public AlphaBuffer(int length, double alpha) {
        this(length);
        Arrays.fill(buffer, alpha);
    }

    public static AlphaBuffer linear(int length) {
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            result[i] = (double) i / (length - 1);
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer sine(int length, double period, double phase) {
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            result[i] = (1 - Math.cos(2 * Math.PI / period * (i - phase))) / 2;
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer invert() {
        double[] result = new double[buffer.length];
        for (int i = 0; i < buffer.length; i++) {
            result[i] = 1 - buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer multiply(int outputLength, AlphaBuffer a, AlphaBuffer b) {
        double[] result = new double[outputLength];
        int contentLength = Math.min(outputLength, Math.min(a.buffer.length, b.buffer.length));
        for (int i = 0; i < contentLength; i++) {
            result[i] = a.buffer[i] * b.buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer layer(int outputLength, AlphaBuffer... layers) {
        double[] result = new double[outputLength];
        for (AlphaBuffer layer : layers) {
            if (layer == null) continue;
            for (int i = 0; i < Math.min(outputLength, layer.buffer.length); i++) {
                result[i] = result[i] + layer.buffer[i] * (1 - result[i]);
            }
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer concat(int outputLength, AlphaBuffer... segments) {
        double[] result = new double[outputLength];
        int i = 0;
        for (AlphaBuffer segment : segments) {
            if (segment == null) continue;
            for (int j = 0; j < segment.buffer.length; j++) {
                if (i >= outputLength) return new AlphaBuffer(result);
                result[i] = segment.buffer[j];
                i++;
            }
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer concat(AlphaBuffer... segments) {
        int length = 0;
        for (AlphaBuffer segment : segments) {
            if (segment == null) continue;
            length += segment.buffer.length;
        }
        return concat(length, segments);
    }

    public AlphaBuffer crop(int length) {
        double[] result = new double[length];
        for (int i = 0; i < Math.min(length, buffer.length); i++) {
            result[i] = buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer flip() {
        double[] result = new double[buffer.length];
        for (int i = 0; i < buffer.length; i++) {
            result[i] = buffer[buffer.length - i - 1];
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer tile(int length) {
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            int j = Math.floorMod(i, buffer.length);
            result[i] = buffer[j];
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer repeat(int times) {
        return tile(times * buffer.length);
    }

    public AlphaBuffer wrappedShift(int outputLength, int offset) {
        double[] result = new double[outputLength];
        for (int i = 0; i < outputLength; i++) {
            result[i] = buffer[Math.floorMod(i - offset, buffer.length)];
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer cycle(int offset) {
        return wrappedShift(buffer.length, offset);
    }

    public AlphaBuffer shift(int outputLength, int offset) {
        double[] result = new double[outputLength];
        for (int i = Math.max(0, -offset); i < Math.min(buffer.length, outputLength - offset); i++) {
            int j = i + offset;
            result[j] = buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public AlphaBuffer multiply(AlphaBuffer other) {
        return multiply(buffer.length, this, other);
    }

    public AlphaBuffer layer(AlphaBuffer other) {
        return layer(buffer.length, this, other);
    }

    public AlphaBuffer append(AlphaBuffer other) {
        return concat(this, other);
    }

    public AlphaBuffer prepend(AlphaBuffer other) {
        return concat(other, this);
    }
}
