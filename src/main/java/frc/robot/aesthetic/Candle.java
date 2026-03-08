package frc.robot.aesthetic;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class Candle {
    public enum Effect {
        LARSON,
        FLOW,
        CHROMA,
        BREATHE,
        REN_SPECIAL,
        EW,
    }

    private static final CANdle candle = new CANdle(LightsConst.ID);

    static {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        config.LED.BrightnessScalar = LightsConst.BRIGHTNESS;
        candle.getConfigurator().apply(config);
    }

    public ControlRequest getEffectAnimation(Effect effect) {
        int end = LightsConst.LED_COUNT - 1;
        return switch (effect) {
            case LARSON -> new LarsonAnimation(0, end).withColor(new RGBWColor(255, 255, 255));
            case FLOW -> new ColorFlowAnimation(0, end).withColor(new RGBWColor(255, 255, 255));
            case CHROMA -> new RainbowAnimation(0, end).withBrightness(0.2).withFrameRate(10.0);
            case BREATHE -> new SingleFadeAnimation(0, end).withColor(new RGBWColor(235, 209, 39));
            case REN_SPECIAL -> new TwinkleAnimation(0, end).withColor(new RGBWColor(235, 209, 39));
            case EW ->
                    new SingleFadeAnimation(0, end)
                            .withColor(new RGBWColor(198, 184, 54)); // disgusting color
        };
    }

    public void animateCandle(Effect effect) {
        candle.setControl(getEffectAnimation(effect));
    }

    public void staticColor(int red, int green, int blue) {
        candle.setControl(
                new SolidColor(0, LightsConst.LED_COUNT - 1)
                        .withColor(new RGBWColor(red, green, blue)));
    }

    public void staticColor(Color8Bit color) {
        candle.setControl(
                new SolidColor(0, LightsConst.LED_COUNT - 1).withColor(new RGBWColor(color)));
    }
}
