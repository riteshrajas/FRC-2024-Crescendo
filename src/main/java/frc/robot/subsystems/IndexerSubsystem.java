package frc.robot.subsystems;

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
        IndexerMotor.getConfigurator().apply(new TalonFXConfiguration());
    }
   
    public boolean IndexerHasNote()  //TODO: Finish this code when we have the sensors and know how to use them
    {
        return true; //placeholder
    }


    public boolean IntakeCorrectPose() //TODO: determine logic of code when we know how Note will enter indexer
    {
        return true; //placeholder
    }

    public Command Command_ActuateIndexer() 
    {
        return run(() -> {
            if (IndexerHasNote() && IntakeCorrectPose()) {
                IndexerMotor.set(Constants.Indexer.Indexer_MOTOR_POWER);
            }
        });
    }


}
