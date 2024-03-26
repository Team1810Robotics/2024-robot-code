package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Shoot extends ParallelCommandGroup {
    public Shoot(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            ArmSubsystem arm,
            VisionSubsystem vision) {
        boolean block = !vision.isAligned() && !arm.atSetpoint();
        addCommands(new ShooterCommand(shooter, intake, true, block), new AimCommand(vision, arm));
    }
}
