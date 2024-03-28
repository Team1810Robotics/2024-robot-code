package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;

public class Shoot extends ParallelCommandGroup {

    public Shoot(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            ArmSubsystem arm,
            VisionSubsystem vision) {
        BooleanSupplier block = () -> !vision.isAligned() && !arm.atSetpoint();
        addCommands(
                arm.setpointCommand(vision.getAngle()),
                new ShooterCommand(shooter, intake, true, block));
    }
}
