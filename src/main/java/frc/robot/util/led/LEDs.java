// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package frc.robot.util.led;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.ColorConstants;
import frc.robot.Constants.LEDConstants.LengthConstants;

public class LEDs extends SubsystemBase {
    //#region SINGLETON, SETUP, AND CONSTRUCTOR

    private static LEDs instance = null;

    public static LEDs getInstance() {
        return instance;
    }

    AddressableLED led = new AddressableLED(LEDConstants.LED_PORT);

    public LEDs() {
        if (instance != null) throw new IllegalStateException("LEDs already initialized");
        instance = this;

        led.setLength(LengthConstants.TOTAL);
        led.start();

       loadingNotifier.startPeriodic(0.02);
    }

    //#endregion
    //#region STATE VARIABLES

    public enum ConnectionState { DISCONNECTED, DS_ONLY, FMS }
    ConnectionState connectionState = ConnectionState.DISCONNECTED;
    DriverStation.Alliance alliance = DriverStation.Alliance.Red;
    public enum RobotMode { DISABLED, TELEOP, AUTONOMOUS, ESTOPPED }
    RobotMode robotMode = RobotMode.DISABLED;
    boolean aligned = false;
    boolean aligning = false;
    boolean readyForIntake = false;
    boolean hasCoral = false;
    boolean userSignal = false;

    public Color debugColor = null;
    LoggedNetworkNumber debugLength = new LoggedNetworkNumber("/Tuning/LED/DebugLength", 0);

    //#endregion
    //#region STATE GETTERS AND SETTERS

    void setFMSState(ConnectionState value) { connectionState = value; }
    void setAlliance(DriverStation.Alliance value) { alliance = value; }
    public void setAligned(boolean aligned) { this.aligned = aligned; }
    public void setAligning(boolean aligning) { this.aligning = aligning; }
    public void setReadyForIntake(boolean readyForIntake) { this.readyForIntake = readyForIntake; }
    public void setHasCoral(boolean hasCoral) { this.hasCoral = hasCoral; }
    public void setUserSignal(boolean userSignal) { this.userSignal = userSignal; }
    void triggerTeleopCountdown() {}
    void triggerEndgameSignal() {}

    void setRobotMode(RobotMode value) {
        if (value == RobotMode.DISABLED && robotMode == RobotMode.AUTONOMOUS
                && connectionState == ConnectionState.FMS) {
            triggerTeleopCountdown();
        }
        robotMode = value;
    }


    //#endregion

    public void periodic() {
        loadingNotifier.stop();

        // Sets driver station state variables to reflect realtiy 
        if (!DriverStation.isDSAttached()) {
            setFMSState(ConnectionState.DISCONNECTED);
        } else if (DriverStation.isFMSAttached()) {
            setFMSState(ConnectionState.FMS);
            if (DriverStation.getAlliance().isPresent()) { setAlliance(DriverStation.getAlliance().get()); }
        } else {
            setFMSState(ConnectionState.DS_ONLY);
        }

        if (DriverStation.isEStopped()) {
            setRobotMode(RobotMode.ESTOPPED);
        } else if (DriverStation.isAutonomousEnabled()) {
            setRobotMode(RobotMode.AUTONOMOUS);
        } else if (DriverStation.isTeleopEnabled()) {
            setRobotMode(RobotMode.TELEOP);
        } else {
            setRobotMode(RobotMode.DISABLED);
        }

        // Updates LEDs
        led.setData(
                LobstahLEDBuffer
                        .layer(LengthConstants.TOTAL,
                                robotMode == RobotMode.DISABLED
                                        ? connectionState == ConnectionState.DISCONNECTED ? disconnected() // Disconnected
                                                : disabledStandby() //Standby
                                        : null,
                                robotMode == RobotMode.AUTONOMOUS ? autonomous() : null, //Auto
                                aligning ? LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kBlue).opacity(.5) : null,
                                aligned ? LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kLime) : null,
                                readyForIntake ? prideFlagCycle(LengthConstants.TOTAL, 8) : null,
                                // hasCoral ? LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kPurple) : null,
                                userSignal ? LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kWhite) : null,
                                debugColor == null ? null : LobstahLEDBuffer.solid(LengthConstants.TOTAL, debugColor), //for testing
                                LobstahLEDBuffer.solid((int)debugLength.get(), Color.kWhite)
                              ).toAdressableLEDBuffer());
    }

    //#region PATTERNS
    //#region INSTANCES OF PATTERN-RELATED OBJECTS
    // Timer possessionSignalTimer = new Timer();

    final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            led.setData(loading().toAdressableLEDBuffer());
        }
    });

    //#endregion

    static LobstahLEDBuffer segments(LobstahLEDBuffer left, LobstahLEDBuffer midSegment, LobstahLEDBuffer right) {
        return LobstahLEDBuffer.concat(
                left == null ? new LobstahLEDBuffer(LengthConstants.LEFT) : left.crop(LengthConstants.LEFT),
                midSegment == null ? new LobstahLEDBuffer(LengthConstants.MID) : midSegment.crop(LengthConstants.MID),
                right == null ? new LobstahLEDBuffer(LengthConstants.RIGHT) : right.crop(LengthConstants.RIGHT).flip());
    }

    static LobstahLEDBuffer loading() {
        double opacity = AnimationEasing.sine(System.currentTimeMillis(), 1000, 0);
        LobstahLEDBuffer buffer = LobstahLEDBuffer.solid(10, ColorConstants.LOADING).mask(AlphaBuffer.sine(10, 20, -10))
                .opacity(opacity);
        return segments(buffer, null, buffer);
    }

    static LobstahLEDBuffer disconnected() {
        int bouncyBallLength = 3;
        int bouncyBallOffset = (int) (AnimationEasing.sine(Timer.getFPGATimestamp(), 1.5, 0) * LengthConstants.MID
                - bouncyBallLength / 2);
        LobstahLEDBuffer bouncyBall = LobstahLEDBuffer.solid(3, ColorConstants.LOADING).shift(LengthConstants.MID,
                bouncyBallOffset);

        int waveLength = 10;
        LobstahLEDBuffer waves = LobstahLEDBuffer.solid(waveLength, ColorConstants.LOADING).mask(
                AlphaBuffer.sine(waveLength, waveLength, AnimationEasing.sine(Timer.getFPGATimestamp(), 3, 0) * 10));

        return segments(waves.tile(LengthConstants.LEFT), bouncyBall,
                waves.cycle(-LengthConstants.RIGHT).tile(LengthConstants.RIGHT));
    }

    static LobstahLEDBuffer disabledStandby() {
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, new Color("#FF5555"), 0.5)
                    .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 10, Timer.getFPGATimestamp() * 20))
                    .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kRed));
    }

    static LobstahLEDBuffer autonomous() {
        return LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_1, 0.5)
                .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 10, Timer.getFPGATimestamp() * 10))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_2)
                        .mask(AlphaBuffer.sine(LengthConstants.TOTAL, 5, Timer.getFPGATimestamp() * 7)))
                .layerAbove(LobstahLEDBuffer.solid(LengthConstants.TOTAL, ColorConstants.AUTON_3));
    }

    static LobstahLEDBuffer prideFlagCycle(int segmentLength, double speed) {
        int offset = (int) (Timer.getFPGATimestamp() * speed);
        return prideFlag(segmentLength).prepend(LobstahLEDBuffer.solid(1, Color.kBlack)).cycle(offset);

    }

    static LobstahLEDBuffer prideFlag(int segmentLength) {
        return LobstahLEDBuffer.concat(LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_RED),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_ORANGE),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_YELLOW),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_GREEN),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_BLUE),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.PRIDE_PURPLE),
                LobstahLEDBuffer.solid(1, Color.kBlack),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_TEAL),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_PINK),
                LobstahLEDBuffer.solid(segmentLength, Color.kWhite),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_PINK),
                LobstahLEDBuffer.solid(segmentLength, ColorConstants.TRANS_TEAL)).flip();
    }
    //#endregion
}