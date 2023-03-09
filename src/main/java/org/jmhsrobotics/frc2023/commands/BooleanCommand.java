package org.jmhsrobotics.frc2023.commands;

import java.net.CookieHandler;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BooleanCommand extends CommandBase {
    BooleanSupplier booleanSupplier;
    CommandBase command1;
    CommandBase command2;
    public BooleanCommand(BooleanSupplier booleanSupplier, CommandBase command1, CommandBase command2) {
        this.booleanSupplier = booleanSupplier;
        this.command1 = command1;
        this.command2 = command2;
    }
    public void execute() {
        if (booleanSupplier.getAsBoolean()) { 
            command1.schedule();
        } else {
            command2.schedule();
        }
    }
    public boolean isFinished() {
        return true;
    }
}
