package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase
{
    private TalonFX IndexerMotor = new TalonFX(Constants.Indexer.INDEXER_MOTOR_ID);

    public IndexerSubsystem()
    {
        ConfigureMotors(IndexerMotor, 0, 0, 0, 0);
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

   
    public boolean IndexerHasNote()  //TODO: Finish this code when we have the sensors and know how to use them
    {
        return true; //placeholder
    }


    public Command Command_ActuateIndexer() 
    {
        return run(() -> {
            if (IndexerHasNote()) {
                IndexerMotor.set(Constants.Indexer.Indexer_MOTOR_POWER);
            }
        });
    }


}
