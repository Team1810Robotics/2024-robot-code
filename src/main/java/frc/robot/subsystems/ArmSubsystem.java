package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem {

    public ArmSubsystem() {
        super(ArmConstants.CONSTRAINTS, ArmConstants.INITIAL_POSITION);
    }

    @Override
    public void useState(State state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useState'");
    }
    
}
