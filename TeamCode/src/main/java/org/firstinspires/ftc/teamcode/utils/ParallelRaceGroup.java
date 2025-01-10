package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * A CommandGroup that runs a set of commands in parallel, ending when any one of the commands ends
 * and interrupting all the others.
 *
 * <p>As a rule, CommandGroups require the union of the requirements of their component commands.
 *
 */
public class ParallelRaceGroup extends CommandGroupBase {

    private final Map<Command, Boolean> m_commands = new HashMap<>();
    private boolean m_runWhenDisabled = true;
    private boolean m_finished = true;

    /**
     * Creates a new ParallelCommandRace. The given commands will be executed simultaneously, and
     * will "race to the finish" - the first command to finish ends the entire command, with all other
     * commands being interrupted.
     *
     * @param commands the commands to include in this group.
     */
    public ParallelRaceGroup(Command... commands) {
        addCommands(commands);
    }

    @Override
    public final void addCommands(Command... commands) {
        requireUngrouped(commands);

        if (!m_finished) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        // So ridiculous that this function isn't public
//        registerGroupedCommands(commands);

        for (Command command : commands) {
            if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
                throw new IllegalArgumentException("Multiple commands in a parallel group cannot"
                        + " require the same subsystems");
            }
            m_commands.put(command, false);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        m_finished = false;
        for (Command command : m_commands.keySet()) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        for (Command command : m_commands.keySet()) {
            command.execute();
            if (command.isFinished()) {
                m_commands.replace(command, true);
                m_finished = true;
                command.end(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (Command command : m_commands.keySet()) {
            // Have you considered combing this with SequentialCommandGroup?
            if (Boolean.FALSE.equals(m_commands.get(command)) && !command.isFinished()) {
                command.end(true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

}
