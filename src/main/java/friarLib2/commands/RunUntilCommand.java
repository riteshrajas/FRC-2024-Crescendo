package friarLib2.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Run the specified command until the specified condition returns true or
 * until that command finishes
 */
public class RunUntilCommand extends ParallelRaceGroup {
    public RunUntilCommand(Command command, BooleanSupplier endCondition) {
        addCommands(
            command, 
            new WaitUntilCommand(endCondition)
        );
    }
}
