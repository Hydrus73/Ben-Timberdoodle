// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  AddressableLED lights;
  AddressableLEDBuffer ledBuffer;
  boolean runRainbow;
  int startHue;
  /** Creates a new Lights. */
  public Lights() {
    lights = new AddressableLED(Constants.LightsConstants.PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.LightsConstants.COUNT);
    startHue = 0;
    runRainbow = false;

    lights.setLength(ledBuffer.getLength());
    lights.setData(ledBuffer);
    lights.start();
  }

  public void rainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, (startHue + 2*i) % 180, 200, 100);
    }
    lights.setData(ledBuffer);
    startHue = (startHue + 2) % 180;
  }

  public void setRainbow(boolean on) {
    runRainbow = on;
  }

  public void cone() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 247, 227, 42);
    }
    lights.setData(ledBuffer);
  }

  public void cube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 159, 24, 237);
    }
    lights.setData(ledBuffer);
  }

  public void stop() {
    lights.stop();
  }

  public void start() {
    lights.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runRainbow) {
      rainbow();
    }
  }
}
