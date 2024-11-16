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
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;
        wrist = drive.wrist;
    }

    public void Initialize() {
        //initialize four bar to transfer
        RotateFourBar(1);
        wrist.setPosition(0.2);
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
        int currentPosition = armRight.getCurrentPosition();
        //Math.clamp causes crash here, so using custom method
        int targetPosition = (int)Math.max(-120, Math.min(currentPosition + power * 10, 1200));
        if (currentPosition != targetPosition) {
            GoToHeight(targetPosition);
            armRight.setPower(power == 0 ? Math.signum(targetPosition - currentPosition) : power);
            armLeft.setPower(power == 0 ? 1 : power);
        }
        else if (armRight.getPower() != 0) {
            armRight.setPower(0);
            armLeft.setPower(0);
        }

        telemetry.addData("liftPos", armRight.getCurrentPosition());
        telemetry.addData("liftTarget", targetPosition);

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
