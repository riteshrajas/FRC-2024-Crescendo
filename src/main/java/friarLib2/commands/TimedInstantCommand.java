package friarLib2.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Like an InstantCommand, but will not finish until the specified amount of
 * time passes.
 */
public class TimedInstantCommand extends ParallelDeadlineGroup {
    /**
     * Create a new TimedInstantCommand.
     * 
     * @param timeLimitSeconds The amount of time before the command ends
     * @param toRun The action to execute
     * @param requirements The required subsystems
     */
    public TimedInstantCommand (double timeLimitSeconds, Runnable toRun, Subsystem... requirements) {
        super(
            new WaitCommand(timeLimitSeconds), 
            new InstantCommand(toRun, requirements)
        );
    }
}
