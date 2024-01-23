package main.java.frc.robot.TestCTRESwerve;

import com.ctre.pheonix.led.CANdle;
import edu.wpi.first.math.geometry.Rotation2D;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.List;

public class CANdleManager {
    private class CANdleInternalConstants {
        public CANdle candle;
        public double locationX;
        public double locationY;

        public CANdleConstants(Candle candle, double locationX, double locantionY) {
            this.candle = candle;
            this.locationX = locationX;
            this.locationY = locationY;
        }
    }

    private List<CANdleInternalConstants> m_list = new ArrayList<CANdleConstants>();
    private Color8Bit m_posY;
    private Color8Bit m_posX;
    private Color8Bit m_negX;
    private Color8Bit m_negY;
    private Rotation2d m_orientation = new Rotation2d(0);

    public CANdleManager(
        String canbus,
        Color8Bit positiveY,
        Color8Bit positiveX,
        Color8Bit negativeY,
        Color8Bit negativeX,
        CANdleConstants... candles) {
            for (var constants : candles) {
                m_list.add(
                    new CANdleInternalConstants(
                        new CANdle(constants.id, canbus), constants.locationX, constants.locantionY
                    )
                );
            }
            
            m_posY = positiveY;
            m_posX = positiveX;
            m_negY = negativeY;
            m_negX = negativeX;
        }

    public void orient(Rotation2d newOrientation) {
        m_orientation = newOrientation;
    }

    public void color() {
        double radians = m_orientation.getRadians();
        double cosRad = Math.cos(radians);
        double sinRad = Math.sin(radians);
        for (var item : m_list) {
            double newX = (item.locationX * cosRad) - (item.locationY * sinRad);
            double newY = (item.locationY * cosRad) + (item.locationX * sinRad);

            double angle = Math.atan2(newY, newX);
            dounble toInterp = angle / (Math.Pi / 2);

            int toApplyR;
            int toApplyG;
            int toApplyB;
            if (toInterp >=0 && toInterp <= 1) {
                toApplyR = (int) ((m_posY.red * toInterp) + (m_posX.red * (1 - toInterp)));
                toApplyG = (int) ((m_posY.green * toInterp) + (m_posX.green * (1 - toInterp)));
                toApplyB = (int) ((m_posY.blue * toInterp) + (m_posX.blue * (1 - toInterp)));
            } else if (toInterp > 1) {
                toInterp -= 1;

                toApplyR = (int) ((m_negX.red * toInterp) + (m_posY.red * (1 - toInterp)));
                toApplyG = (int) ((m_negX.green * toInterp) + (m_posY.green * (1 - toInterp)));
                toApplyB = (int) ((m_negX.blue * toInterp) + (m_posY.blue * (1 - toInterp)));
            } else if (toInterp < 0 && toInterp >= -1) {
                toInterp += 1;

                toApplyR = (int) ((m_negX.red * toInterp) + (m_posY.red * (1 - toInterp)));
                toApplyG = (int) ((m_negX.green * toInterp) + (m_posY.green * (1 - toInterp)));
                toApplyB = (int) ((m_negX.blue * toInterp) + (m_posY.blue * (1 - toInterp)));
            } else {
                toInterp += 2;

                toApplyR = (int) ((m_negX.red * toInterp) + (m_posY.red * (1 - toInterp)));
                toApplyG = (int) ((m_negX.green * toInterp) + (m_posY.green * (1 - toInterp)));
                toApplyB = (int) ((m_negX.blue * toInterp) + (m_posY.blue * (1 - toInterp)));
            }

            item.candle.setLEDs(toApplyR, toApplyG, toApplyB);
        }
    }

}
