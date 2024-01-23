package main.java.frc.robot.TestCTRESwerve;

public class SwerveDriveTrainConstants {
    public int Pigeon2Id = 0;

    public String CANbusName = "rio";
    
    public double TurnKp = 0;
    public double TurnKd = 0;

    public SwerveDriveTrainConstants withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    public SwerveDriveTrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKp(double TurnKp) {
        this.TurnKp = TurnKp;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKd(double TurnKd) {
        this.TurnKd = TurnKd;
        return this;
    }
}
