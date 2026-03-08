package frc.robot.aesthetic.candle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;

public final class CandleConfig {
    public static final CANdleConfiguration config = new CANdleConfiguration();

    public static final double BRIGHTNESS = 0.5;

    static {
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = BRIGHTNESS;
    }
}
