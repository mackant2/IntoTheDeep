package org.firstinspires.ftc.teamcode.util.custom;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Logger;
import org.firstinspires.ftc.teamcode.components.TransferPlate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public Arm arm;
    public Drivetrain drivetrain;
    public Intake intake;
    public Logger logger;
    public TransferPlate transferPlate;
    public LinearOpMode opMode;
    public SampleMecanumDrive drive;

    void Initialize() {
        //initialize four bar
        arm.Initialize();
        //flip intake up and bring in
        intake.Initialize();
    }

    public Robot(LinearOpMode opMode, SampleMecanumDrive drive) {
        this.drive = drive;
        this.opMode = opMode;
        arm = new Arm(this);
        drivetrain = new Drivetrain(this);
        logger = new Logger();
        logger.Initialize(opMode.telemetry);
        intake = new Intake(drive, this);
        transferPlate = new TransferPlate(this);

        Initialize();
    }

    public void Update() {
        arm.Update();
        drivetrain.Update();
        intake.Update();
        transferPlate.Update();
    }
}
