package frc.robot.subsystems.Indicator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IndicatorSubsystem extends SubsystemBase {

  private AddressableLED led;
  private static AddressableLEDBuffer ledBuffer;

  private boolean isLightChanged;

  private final int NUM_OF_LEDS = 8;
  private final int PWM_SLOT = 9;
  private int numberOfStatusIndicators;

  @SuppressWarnings("unused")
  private RobotContainer m_RobotContainer;

  public IndicatorSubsystem(RobotContainer robotContainer) {
    this.m_RobotContainer = robotContainer;
    ledBuffer = new AddressableLEDBuffer(NUM_OF_LEDS);
    led = new AddressableLED(PWM_SLOT);
    led.setLength(NUM_OF_LEDS);
    led.start();
    setAllLeds(0, 0, 128); // Set all LEDs to Blue at start up
    // numberOfLimitsSwitches = robotContainer.getLimitSwitches().length;
    numberOfStatusIndicators = 4;
  }

  @Override
  public void periodic() {
    if (Robot.count % 25 == 10) {
      setAllianceColor();
    }
    if (isLightChanged) {
      led.setData(ledBuffer);
    }
    isLightChanged = false;
  }

  public void setAllLeds(int r, int g, int b) {
    for (int i = 0; i < ledBuffer.getLength() - 1; i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    isLightChanged = true;
  }

  // Start the Robot Position Display after the limit switches
  public void setLedsPosition(int r, int g, int b) {
    for (int i = numberOfStatusIndicators + 1; i < ledBuffer.getLength() - 1; i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    isLightChanged = true;
  }

  public void setLEDColorForPositioning(boolean valid) {
    if (valid) {
      setLedsPosition(0, 255, 0);
    } else {
      setLedsPosition(255, 0, 0);
    }
  }

  private void setLedStatusForStatusIndicators(int led, boolean value) {
    if (value) {
      ledBuffer.setRGB(led, 255, 0, 0);
    } else {
      ledBuffer.setRGB(led, 0, 255, 0);
    }
  }

  public void setTransitionLEDColor(boolean valid) {
    setLedsPosition(0, 0, 255);
  }

  public void setStatusIndicators(boolean[] limits) {
    // boolean limits[] = m_RobotContainer.getLimitSwitches();
    int led = 0;
    for (boolean value : limits) {
      setLedStatusForStatusIndicators(led, value);
      led++;
    }
  }

  public void setAllianceColor() {
    if (Robot.isAllianceBlue()) {
      ledBuffer.setRGB(3, 0, 0, 255);
    } else {
      ledBuffer.setRGB(3, 255, 0, 0);
    }
  }
}
