package org.firstinspires.ftc.teamcode.util.custom;

import com.acmerobotics.roadrunner.trajectory.config.TrajectoryGroupConfig;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ParsedHardwareMap {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx liftLeft, liftRight, extender, intake;
    public Servo claw, leftFourBar, rightFourBar, wrist, flipDown, gate;
    public RevBlinkinLedDriver display;
    public ColorSensor leftColorSensor, rightColorSensor, transferSensor;
    public DistanceSensor leftDistanceSensor, rightDistanceSensor, transferDistanceSensor;
    public TouchSensor liftLimiter, extenderLimiter;

    public ParsedHardwareMap(HardwareMap hardwareMap) {
        //Motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "left_back");
        backLeft = hardwareMap.get(DcMotorEx.class, "right_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_front");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        extender = hardwareMap.get(DcMotorEx.class, "Extendo");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        //Servos
        claw = hardwareMap.get(Servo.class, "claw");
        leftFourBar = hardwareMap.get(Servo.class, "leftFourBar");
        rightFourBar = hardwareMap.get(Servo.class, "rightFourBar");
        wrist = hardwareMap.get(Servo.class, "wrist");
        flipDown = hardwareMap.get(Servo.class, "flipdown");
        gate = hardwareMap.get(Servo.class, "gate");

        //Sensors
        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");
        transferSensor = hardwareMap.get(ColorSensor.class, "transferSensor");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftColorSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightColorSensor");
        transferDistanceSensor = hardwareMap.get(DistanceSensor .class, "transferSensor");
        liftLimiter = hardwareMap.get(TouchSensor.class, "liftLimiter");
        extenderLimiter = hardwareMap.get(TouchSensor.class, "extenderLimiter");

        //LEDs
        display = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //Configure Drivetrain
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Configure Lift
        liftLeft.setTargetPosition(liftLeft.getCurrentPosition());
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFourBar.setDirection(Servo.Direction.FORWARD);
        rightFourBar.setDirection(Servo.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftLeft.setVelocity(1000);

        //Configure Intake
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setTargetPosition(extender.getCurrentPosition());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setDirection(DcMotorSimple.Direction.REVERSE);
        extender.setVelocity(1000);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
