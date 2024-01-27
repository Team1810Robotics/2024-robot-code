package frc.robot.commands;
 
 public class ShooterCommand extends ShooterCommand{
    
    
    ShooterSubsystem ShooterSubsystem;

    double launch;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double launch){
        this.motor = launch;
        this.shooterSubsystem = ShooterSubsystem;

        addRequirments(shooterSubsystem);
    }

    public void execute(){
       shooterSubsystem.set(launch);
    }

    public void end(){
        shooterSubssytem.set(0);
    }
}
