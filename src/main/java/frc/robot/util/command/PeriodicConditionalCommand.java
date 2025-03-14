// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.command;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

/**
 * A command composition that runs one of two commands, depending on the value of the given condition when this command
 * is initialized.
 *
 * <p>
 * The rules for command compositions apply: command instances that are passed to it cannot be added to any other
 * composition or scheduled individually, and the composition requires all subsystems its components require.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class PeriodicConditionalCommand extends Command {
  private final Command m_onTrue;
  private final Command m_onFalse;
  private final BooleanSupplier m_condition;
  private Command m_selectedCommand;

  /**
   * Creates a new ConditionalCommand.
   *
   * @param onTrue the command to run if the condition is true
   * @param onFalse the command to run if the condition is false
   * @param condition the condition to determine which command to run
   */
  public PeriodicConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
    m_onTrue = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
    m_onFalse = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
    m_condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

    CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);

    addRequirements(m_onTrue.getRequirements());
    addRequirements(m_onFalse.getRequirements());
  }

  @Override
  public void initialize() {
    if (m_condition.getAsBoolean()) {
      m_selectedCommand = m_onTrue;
    } else {
      m_selectedCommand = m_onFalse;
    }
    m_selectedCommand.initialize();
  }

  @Override
  public void execute() {
    if (m_condition.getAsBoolean() && m_selectedCommand != m_onTrue) {
      m_selectedCommand = m_onTrue;
      m_onFalse.cancel();
      m_selectedCommand.initialize();
    } else if (!m_condition.getAsBoolean() && m_selectedCommand != m_onFalse) {
      m_selectedCommand = m_onFalse;
      m_onTrue.cancel();
      m_selectedCommand.initialize();
    }

    m_selectedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_onTrue.runsWhenDisabled() && m_onFalse.runsWhenDisabled();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("onTrue", m_onTrue::getName, null);
    builder.addStringProperty("onFalse", m_onFalse::getName, null);
    builder.addStringProperty(
        "selected",
        () -> {
          if (m_selectedCommand == null) {
            return "null";
          } else {
            return m_selectedCommand.getName();
          }
        },
        null);
  }
}
