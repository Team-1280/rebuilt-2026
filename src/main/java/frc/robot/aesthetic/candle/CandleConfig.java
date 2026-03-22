package frc.robot.aesthetic.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;

public final class CandleConfig {
    public static final int LEDStartIndex = 0;
    public static final int LEDEndIndex = CandleConst.LED_COUNT - 1;

    public static final double BRIGHTNESS = 0.5;
    public static final CANdleConfiguration config = new CANdleConfiguration();

    static {
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = BRIGHTNESS;
    }
}
