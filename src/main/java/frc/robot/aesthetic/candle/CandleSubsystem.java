package frc.robot.aesthetic.candle;

import static frc.robot.aesthetic.candle.CandleConfig.LEDEndIndex;
import static frc.robot.aesthetic.candle.CandleConfig.LEDStartIndex;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsystem extends SubsystemBase {

    private static final CANdle candle = new CANdle(CandleConst.ID);

    public CandleSubsystem() {
        candle.getConfigurator().apply(CandleConfig.config);
    }

    public void animateCandle(CandleEffect effect) {
        candle.setControl(effect.animation);
    }

    public void staticColor(int red, int green, int blue) {
        staticColor(new Color8Bit(red, green, blue));
    }

    public void staticColor(Color8Bit color) {
        candle.setControl(
                new SolidColor(LEDStartIndex, LEDEndIndex).withColor(new RGBWColor(color)));
    }
}
