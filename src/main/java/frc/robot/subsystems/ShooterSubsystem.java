package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


import java.io.ObjectInputFilter;

public class ShooterSubsystem {

    private TalonFX ShooterMotor1 = new TalonFX(Constants.Shooter.SHOOTER1_MOTOR_ID);
    private TalonFX ShooterMotor2 = new TalonFX(Constants.Shooter.SHOOTER2_MOTOR_ID);
    public ShooterSubsystem()
    {
        ConfigureMotors(ShooterMotor1, 0, 0,0,0); //TODO: tune me
        ConfigureMotors(ShooterMotor2,0,0,0,0); //TODO: tune me
    }
    private void ConfigureMotors(TalonFX motor, double kV, double kP, double kI, double kD)
    {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = kV;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        // apply gains, 50 ms total timeout
        motor.getConfigurator().apply(slot0Configs, 0.050);
    }


    
}
