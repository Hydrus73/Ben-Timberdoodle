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
  int startRainbowHue;
  boolean runCube;
  boolean runCone;
  int startDefaultIndex;
  boolean bad;
  boolean good;
  /** Creates a new Lights. */
  public Lights() {
    lights = new AddressableLED(Constants.LightsConstants.PORT);
    ledBuffer = new AddressableLEDBuffer(Constants.LightsConstants.COUNT);
    startRainbowHue = 0;
    runRainbow = false;
    runCube = false;
    runCone = false;
    startDefaultIndex = 0;
    bad = false;
    good = false;

    lights.setLength(ledBuffer.getLength());
    lights.setData(ledBuffer);
    lights.start();
  }

  public void rainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, (startRainbowHue + 2*i) % 180, 200, 100);
    }
    lights.setData(ledBuffer);
    startRainbowHue = (startRainbowHue + 2) % 180;
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

  public void setCone(boolean on) {
    runCone = on;
  }

  public void cube() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 159, 24, 237);
    }
    lights.setData(ledBuffer);
  }

  public void setCube(boolean on) {
    runCube = on;
  }

  public void stop() {
    lights.stop();
  }

  public void start() {
    lights.start();
  }

  public void defaultColor() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + startDefaultIndex)/3 % 2 == 0) {
        ledBuffer.setRGB(i, 252, 236, 3);
      }
      else {
        ledBuffer.setRGB(i, 10, 10, 245);
      }
    }
    lights.setData(ledBuffer);
    startDefaultIndex++;
  }

  public void bad() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 250, 10, 10);
    }
    lights.setData(ledBuffer);
    startDefaultIndex++;
  }

  public void setBad(boolean on) {
    bad = on;
  }

  public void good() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 10, 250, 10);
    }
    lights.setData(ledBuffer);
    startDefaultIndex++;
  }

  public void setGood(boolean on) {
    good = on;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runRainbow) {
      rainbow();
    }
    else if (runCone) {
      cone();
    }
    else if (runCube) {
      cube();
    }
    else if (bad) {
      bad();
    }
    else if (good) {
      good();
    }
    else {
      defaultColor();
    }
  }
}
