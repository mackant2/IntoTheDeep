package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.util.custom.EnhancedColorSensor;
import org.firstinspires.ftc.teamcode.util.custom.Robot;

public class TransferPlate {
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    public boolean sampleIsPresent;

    public TransferPlate(Robot robot) {
        colorSensor = robot.drive.transferSensor;
        distanceSensor = robot.drive.transferDistanceSensor;
    }

    public void Update() {
        sampleIsPresent = EnhancedColorSensor.CheckSensor(colorSensor, distanceSensor, EnhancedColorSensor.Color.Any);
    }
}
