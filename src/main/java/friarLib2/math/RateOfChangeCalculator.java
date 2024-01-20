package friarLib2.math;

import edu.wpi.first.wpilibj.Timer;

public class RateOfChangeCalculator {
    
    private double previousValue = 0;
    private double latestValue = 0;

    private double previousTime = 0;
    private double latestTime = 0;

    private Timer timer = new Timer();

    public RateOfChangeCalculator() {
        timer.reset();
        timer.start();
    }

    public double update(double newValue) {
        previousValue = latestValue;
        latestValue = newValue;

        previousTime = latestTime;
        latestTime = timer.get();

        return getRoC();
    }

    public double getRoC() {
        return (latestValue - previousValue) / (latestTime - previousTime);
    }
}
