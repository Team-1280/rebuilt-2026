package frc.robot.aesthetic.candle;

import static frc.robot.aesthetic.candle.CandleConfig.LEDEndIndex;
import static frc.robot.aesthetic.candle.CandleConfig.LEDStartIndex;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

public enum CandleEffect {
    LARSON(new LarsonAnimation(LEDStartIndex, LEDEndIndex).withColor(new RGBWColor(255, 255, 255))),
    FLOW(
            new ColorFlowAnimation(LEDStartIndex, LEDEndIndex)
                    .withColor(new RGBWColor(255, 255, 255))),
    CHROMA(
            new RainbowAnimation(LEDStartIndex, LEDEndIndex)
                    .withBrightness(0.2)
                    .withFrameRate(10.0)),
    BREATHE(
            new SingleFadeAnimation(LEDStartIndex, LEDEndIndex)
                    .withColor(new RGBWColor(235, 209, 39))),
    REN_SPECIAL(
            new TwinkleAnimation(LEDStartIndex, LEDEndIndex)
                    .withColor(new RGBWColor(235, 209, 39))),
    EW(
            new SingleFadeAnimation(LEDStartIndex, LEDEndIndex)
                    .withColor(new RGBWColor(198, 184, 54))); // disgusting color

    protected final ControlRequest animation;

    CandleEffect(ControlRequest animation) {
        this.animation = animation;
    }
}
