package stl.choreo;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.HashMap;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.Filesystem;

public class ChoreoVariables {
    private static HashMap<String, Double> variableCache = new HashMap<>();
    private static HashMap<String, Pose2d> poseCache = new HashMap<>();
    private static boolean INITIALIZED = false;

    private static double getVal(JsonObject obj) {
        return obj.get("val").getAsDouble();
    }

    private static double getVal(JsonElement obj) {
        return getVal(obj.getAsJsonObject());
    }

    private static void initialize() {
        if (INITIALIZED == true) return;
        File choreoFile = new File(Filesystem.getDeployDirectory(), "choreo/2025robot.chor");
        try {
            var reader = new BufferedReader(new FileReader(choreoFile));
            String str = reader.lines().reduce("", (a, b) -> a + b);
            reader.close();
            Gson GSON = new Gson();
            JsonObject wholeChor = GSON.fromJson(str, JsonObject.class);
            JsonObject variables = wholeChor.get("variables").getAsJsonObject();
            JsonObject expressions = variables.get("expressions").getAsJsonObject();
            for (var entry : expressions.entrySet()) {
                variableCache.put(entry.getKey(), getVal(entry.getValue().getAsJsonObject().get("var")));
            }
            JsonObject poses = variables.get("poses").getAsJsonObject();
            for (var entry : poses.entrySet()) {
                JsonObject val = entry.getValue().getAsJsonObject();
                poseCache.put(entry.getKey(), new Pose2d(getVal(val.get("x")), getVal(val.get("y")),
                        Rotation2d.fromRadians(getVal(val.get("heading")))));
            }
        } catch (Exception e) {
            return;
        }
        INITIALIZED = true;
    }

    public static Pose2d getPose(String key) {
        initialize();
        return poseCache.get(key);
    }

    public static double get(String key) {
        initialize();
        return variableCache.get(key).doubleValue();
    }

    public static Distance getLength(String key) {
        return Meters.of(get(key));
    }

    public static LinearVelocity getLinearVelocity(String key) {
        return MetersPerSecond.of(get(key));
    }

    public static LinearAcceleration getLinearAcceleration(String key) {
        return MetersPerSecondPerSecond.of(get(key));
    }

    public static Angle getAngle(String key) {
        return Radians.of(get(key));
    }

    public static Rotation2d getRotation2d(String key) {
        return Rotation2d.fromRadians(get(key));
    }

    public static AngularVelocity getAngularVelocity(String key) {
        return RadiansPerSecond.of(get(key));
    }

    public static AngularAcceleration getAngularAcceleration(String key) {
        return RadiansPerSecondPerSecond.of(get(key));
    }

    public static Time getTime(String key) {
        return Seconds.of(get(key));
    }

    public static Mass getMass(String key) {
        return Kilograms.of(get(key));
    }

    public static Torque getTorque(String key) {
        return NewtonMeters.of(get(key));
    }

    public static MomentOfInertia getMOI(String key) {
        return KilogramSquareMeters.of(get(key));
    }

    public static void deinitialize() {
        variableCache = new HashMap<>();
        poseCache = new HashMap<>();
        INITIALIZED = false;
        System.gc();
    }
}
