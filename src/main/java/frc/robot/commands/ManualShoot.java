package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class ManualShoot extends Command {

    private final IntakeSubsystem intake;

    // private final BooleanSupplier button;

    public ManualShoot(IntakeSubsystem intake, BooleanSupplier button) {
        this.intake = intake;
        // this.button = button;
        // Shuffleboard.getTab("intake").addBoolean("Blocked", blocked);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (intake.hasNote()) {
            intake.setSpeed(1);
        } else {
            intake.setSpeed(0);
            isFinished();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interupted) {}
}
