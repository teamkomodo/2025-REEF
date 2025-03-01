package frc.robot.commands.utilityCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IfElseCommand extends DynamicCommand {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    public IfElseCommand(BooleanSupplier condition, Command onTrue, Command onFalse) {
        this.condition = condition;
        this.onTrue = onTrue;
        this.onFalse = onFalse;
    }

    @Override
    protected Command getCommand() {
        return Commands.runOnce(() -> {
            if (condition.getAsBoolean()) onTrue.schedule();
            else onFalse.schedule();
        });
    }
}