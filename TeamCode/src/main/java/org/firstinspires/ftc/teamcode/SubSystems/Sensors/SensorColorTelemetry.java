package org.firstinspires.ftc.teamcode.SubSystems.Sensors;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simple TeleOp that reports normalized color sensor readings (R/G/B), HSV and alpha to telemetry.
 * Expects a NormalizedColorSensor configured with the name "sensor_color" in the Robot Configuration.
 */
@TeleOp(name = "Sensor: Color Telemetry", group = "Sensor")
public class SensorColorTelemetry extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private View relativeLayout;

    @Override
    public void runOpMode() {
        // Try to find a RelativeLayout on the robot controller app so we can tint the background
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Get the color sensor from the hardware map
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If the sensor supports a controllable light, turn it on for better readings
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        waitForStart();

        final float[] hsvValues = new float[3];

        while (opModeIsActive()) {
            // Read normalized RGBA from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to Android HSV values
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Report RGB (3 decimal places)
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            // Report HSV (Hue may be a larger number)
            telemetry.addData("Hue", "%.3f", hsvValues[0]);
            telemetry.addData("Saturation", "%.3f", hsvValues[1]);
            telemetry.addData("Value", "%.3f", hsvValues[2]);

            // Report Alpha
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            // If the sensor supports distance measurements, report them too
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();

            // Optionally tint the robot controller background to match detected color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });

            idle();
        }
    }
}