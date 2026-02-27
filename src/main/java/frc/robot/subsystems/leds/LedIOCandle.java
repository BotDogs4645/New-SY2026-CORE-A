package frc.robot.subsystems.leds;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class LedIOCandle implements LedIO {
  public CANdle candle = new CANdle(LedConstants.candleCanId);
  private final StatusSignal<Current> current = candle.getOutputCurrent();
  private final StatusSignal<Temperature> temp = candle.getDeviceTemp();
  private final StatusSignal<Integer> fault = candle.getFaultField();

  public LedIOCandle() {
    var candleConfig = new CANdleConfiguration();
    candleConfig.LED.StripType = StripTypeValue.RGBW;

    tryUntilOk(5, () -> candle.getConfigurator().apply(candleConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, current, temp, fault);
    ParentDevice.optimizeBusUtilizationForAll(candle);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    BaseStatusSignal.refreshAll(current, temp, fault);

    inputs.currentAmps = current.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.fault = fault.getValue();
  }
}
