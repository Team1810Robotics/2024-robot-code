package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ArmSubsystem extends TrapezoidProfileSubsystem {

    public ArmSubsystem() {
        // TODO: Make this a constant in Constants.java
        super(new Constraints(0, 0));
    }

    @Override
    public void useState(State state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useState'");
    }
    
}
