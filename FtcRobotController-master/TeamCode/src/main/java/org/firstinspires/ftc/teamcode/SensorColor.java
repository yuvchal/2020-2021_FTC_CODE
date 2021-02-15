package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "Sensor: Tester Color", group = "Sensor")

public class SensorColor extends LinearOpMode
{
  LynxI2cColorRangeSensor color;

  @Override
  public void runOpMode() throws InterruptedException {

    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;

    color = hardwareMap.get(LynxI2cColorRangeSensor.class,"color");

    waitForStart();

    while(opModeIsActive())
    {
      int currentColor = color.argb();
      Color.RGBToHSV((int)(color.red()*SCALE_FACTOR),(int)(color.green()*SCALE_FACTOR),(int)(color.blue()*SCALE_FACTOR),hsvValues);

      telemetry.addData("Alpha",color.alpha());
      telemetry.addData("Red",color.red());
      telemetry.addData("Green",color.green());
      telemetry.addData("Blue",color.blue());
      telemetry.addData("Hue",hsvValues);
      telemetry.addData("argb: ",currentColor);
      telemetry.update();
    }

  }
}