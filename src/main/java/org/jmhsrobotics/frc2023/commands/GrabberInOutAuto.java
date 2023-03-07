package org.jmhsrobotics.frc2023.commands;

import java.util.concurrent.DelayQueue;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.jmhsrobotics.frc2023.subsystems.GrabberMotorSubsystem;
import org.jmhsrobotics.frc2023.subsystems.GrabberSolenoidSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GrabberInOutAuto extends CommandBase {
    private GrabberMotorSubsystem grabberWheel = new GrabberMotorSubsystem();
    private GrabberSolenoidSubsystem grabberPiston = new GrabberSolenoidSubsystem();
    BooleanSupplier intakeIn;
    private double speed;
    private boolean state;
    Timer time = new Timer();

    public GrabberInOutAuto(BooleanSupplier intakeIn, double speed) {
        this.intakeIn = intakeIn;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        grabberWheel.setGrabberMotor(speed * (intakeIn.getAsBoolean() ? -1 : 1));
        grabberPiston.setGrabberIntakeState(state);
        

    }
    @Override
    public void end(boolean interrupted) {
		if (interrupted) {
            grabberWheel.setGrabberMotor(0);
		}
	}

	// isFinished
	@Override
	public boolean isFinished() {
        if (time.hasElapsed(1)){
		return true;
	    }
        return false;
    }
}

