package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimCommand extends Command {

    private final VisionSubsystem vision;
    private final ArmSubsystem arm;

    public AimCommand(VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem) {
        this.vision = visionSubsystem;
        this.arm = armSubsystem;

        addRequirements(visionSubsystem, armSubsystem);
    }

    @Override
    public void execute() {
        arm.setpoint(vision.getAngle());
    }
}
