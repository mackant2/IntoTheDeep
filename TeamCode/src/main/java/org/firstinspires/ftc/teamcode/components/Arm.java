package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Arm {
    Telemetry telemetry;
    Gamepad driverController;
    DcMotorEx armLeft, armRight;
    Servo leftFourBar, rightFourBar, wrist;
    float liftPos = 0;

    public static class Height {
        public static int LOWER_BUCKET = 0;
        public static int UPPER_BUCKET = 0;
        public static int LOWER_BAR = 0;
        public static int UPPER_BAR = 0;
        public static int DOWN = 0;
    }

    void RotateFourBar(double position) {
        leftFourBar.setPosition(position);
        rightFourBar.setPosition(1 - position);
    }

    public Arm(SampleMecanumDrive drive, Telemetry telemetry, Gamepad driverController) {
        this.driverController = driverController;
        this.telemetry = telemetry;

        //assign motors
        armLeft = drive.armLeft;
        armRight = drive.armRight;
        //assign servos
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;
        wrist = drive.wrist;
        telemetry.addData("four", rightFourBar.getPosition());
    }

    public void Initialize() {
        //initialize four bar to transfer
        RotateFourBar(0.25);
        wrist.setPosition(0.2);

        try {
            Thread.sleep(2000);
            RotateFourBar(0.75);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void GoToHeight(int height) {
        armLeft.setTargetPosition(height);
        armRight.setTargetPosition(height);
    }

    public boolean IsMoving() {
        return armLeft.isBusy();
    }

    public void Update() {
        float armUp = driverController.right_trigger;
        float armDown = driverController.left_trigger;
        float power = armUp - armDown;
        liftPos += power;
        telemetry.addData("liftPos", liftPos);
        telemetry.update();
        float powerSign = Math.signum(power);
        //Math.clamp causes crash here, so using custom method
        int pos = (int)Math.max(0, Math.min(armRight.getTargetPosition() + power * 10, 2500));
        if (powerSign != 0) {
            GoToHeight(pos);
        }
        if (armRight.getPower() != powerSign) {
            armRight.setPower(powerSign);
            armLeft.setPower(powerSign);
        }

        boolean leftDpad = driverController.dpad_left;
        boolean rightDpad = driverController.dpad_right;

        double leftPos = leftFourBar.getPosition();
        double change = 0;
        if (leftDpad) {
            change -= 0.01;
        }
        if (rightDpad) {
            change += 0.01;
        }
        //Math.clamp causes crash here, so using custom method
        double leftClamped = Math.max(0, Math.min(leftPos + change, 1));
        if (change != 0) {
            RotateFourBar(leftClamped);
        }
    }
}
