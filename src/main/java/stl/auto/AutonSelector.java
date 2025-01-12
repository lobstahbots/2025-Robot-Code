// Custom implementation incorporating pair key-value mapping of 6328 Mechanical Advantage for on-the-fly updating auton selection code.

package stl.auto;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSelector<V> extends SubsystemBase {
  private static final int maxQuestions = 4;

  private final LoggedDashboardChooser<AutoRoutine<V>> routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private AutoRoutine<V> lastRoutine;
  private LinkedHashMap<String, V> lastResponses;

  public AutonSelector(String key, String defaultName, List<AutoQuestion<V>> defaultQuestions, Supplier<Command> defaultCommands) {
    routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
    AutoRoutine<V> defaultRoutine = new AutoRoutine<V>(defaultName, defaultQuestions, defaultCommands);
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
    lastResponses = new LinkedHashMap<>();

    // Publish questions and choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
    }
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, List<AutoQuestion<V>> questions, Supplier<Command> command) {
    if (questions.size() > maxQuestions) {
      throw new RuntimeException(
          "Auto routine contained more than "
              + Integer.toString(maxQuestions)
              + " questions: "
              + name);
    }
    routineChooser.addOption(name, new AutoRoutine<V>(name, questions, command));
  }

  /** Returns the selected auto command. */
  public Command getCommand() {
    return lastRoutine.command().get();
  }

  /** Returns the selected question responses. */
  public List<V> getResponses() {
    return new ArrayList<V>(lastResponses.values());
  }

  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
      return;
    }

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }
    if (!selectedRoutine.equals(lastRoutine)) {
      var questions = selectedRoutine.questions();
      for (int i = 0; i < maxQuestions; i++) {
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          questionChoosers
              .get(i)
              .setOptions(
                  questions.get(i).responses().keySet().stream().toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    // Update the routine and responses
    lastRoutine = selectedRoutine;
    lastResponses = new LinkedHashMap<>();
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      String responseString = questionChoosers.get(i).get();
      lastResponses.put(
          responseString == null ? "" : responseString,
          responseString == null
              ? lastRoutine.questions().get(i).responses().values().stream().findFirst().get()
              : lastRoutine.questions().get(i).responses().get(responseString));
    }
  }

  /** A customizable auto routine associated with a single command. */
  private static final record AutoRoutine<V>(
      String name, List<AutoQuestion<V>> questions, Supplier<Command> command) {}

  /** A question to ask for customizing an auto routine. */
  public static record AutoQuestion<V>(String question, Map<String, V> responses) {}
}
