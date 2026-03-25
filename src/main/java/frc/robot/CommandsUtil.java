package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/** Class with further utility methods for commands past what WPILib's Commands class provides. */
public final class CommandsUtil {
    /**
     * Utility method that decorates the command to only run if the command requirements aren't
     * occupied, and to not extend the command's requirements to the command composition.
     */
    public static Command asDefault(Command command) {
        return command.asProxy()
                .onlyIf(
                        () ->
                                command.getRequirements().stream()
                                        .allMatch(
                                                req ->
                                                        CommandScheduler.getInstance()
                                                                        .requiring(req)
                                                                == null));
    }

    /**
     * Utility method that gets a wrapper instant command that schedules the given command and then
     * immediately finishes.
     */
    public static Command instantProxy(Command command) {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(command));
    }
}
