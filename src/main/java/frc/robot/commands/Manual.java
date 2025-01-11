// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.controller.IO.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Manual extends Command {

    private IntakeSubsystem intake;

    /** Creates a new Manual. */
    public Manual(IntakeSubsystem intake) {
        this.intake = intake;
        // Shuffleboard.getTab("intake").addBoolean("Manual", () -> false);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (driver_button3.getAsBoolean()) {
            // Shuffleboard.getTab("intake").addBoolean("Manual", () -> true);
            intake.setSpeed(1);
        }
        if (!driver_button3.getAsBoolean()) {
            // Shuffleboard.getTab("intake").addBoolean("Manual", () -> false);
            intake.setSpeed(0);
        } else {
            // Shuffleboard.getTab("intake").addBoolean("Manual", () -> false);
            intake.setSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!intake.hasNote());
    }
}
