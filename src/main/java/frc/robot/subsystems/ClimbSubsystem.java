package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    
    CANSparkMax leftMotor = new CANSparkMax(Constants.ClimbConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);

    CANSparkMax rightMotor = new CANSparkMax(Constants.ClimbConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

    CANcoder leftBottom = new CANcoder(Constants.ClimbConstants.LEFT_BOTTOM_LS);

    CANcoder rightBottom = new CANcoder(Constants.ClimbConstants.RIGHT_BOTTOM_LS);

    CANcoder leftTop = new CANcoder(Constants.ClimbConstants.LEFT_TOP_LS);

    CANcoder rightTop = new CANcoder(Constants.ClimbConstants.RIGHT_TOP_LS);

    

}
