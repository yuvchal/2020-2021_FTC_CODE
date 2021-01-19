package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Teleop", group="BionicBot")
public class FirstEditionTeleOp extends LinearOpMode {


HardwareBionicbot robot = new HardwareBionicbot();

public void runOpMode() {
    robot.init(hardwareMap);
    waitForStart();

    while (opModeIsActive()) {


        double turn = gamepad1.right_stick_x;
        double lateral = gamepad1.left_stick_x;
        double forward = gamepad1.left_stick_y;
        double leftDrive, leftBack, rightDrive, rightBack;

        leftDrive = forward - lateral + turn;
        leftBack = forward + lateral + turn;
        rightDrive = forward + lateral + turn;
        rightBack = forward - lateral - turn;

        telemetry.addData("Controller X:", gamepad1.left_stick_x);
        telemetry.addData("Controller Y:", gamepad1.left_stick_y);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        //Driving
        if (gamepad1.dpad_left) {
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(.5);
            robot.leftBack.setPower(-.5);
            robot.rightBack.setPower(-.5);
        } else if (gamepad1.dpad_right) {
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(-.5);
            robot.leftBack.setPower(.5);
            robot.rightBack.setPower(.5);
        } else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);
        }
        if(gamepad1.right_stick_x > 0)
        {
            robot.leftDrive.setPower(gamepad1.right_stick_x);
            robot.rightDrive.setPower(gamepad1.right_stick_x);
            robot.rightBack.setPower(gamepad1.right_stick_x);
            robot.leftBack.setPower(gamepad1.right_stick_x);
        }
        else if(gamepad1.right_stick_x < 0)
        {
            robot.leftDrive.setPower(gamepad1.right_stick_x);
            robot.rightDrive.setPower(gamepad1.right_stick_x);
            robot.rightBack.setPower(gamepad1.right_stick_x);
            robot.leftBack.setPower(gamepad1.right_stick_x);
        }
        if (gamepad1.left_trigger > 0)
        {
            robot.leftDrive.setPower(gamepad1.left_trigger);
            robot.rightDrive.setPower(-gamepad1.left_trigger);
            robot.leftBack.setPower(gamepad1.left_trigger);
            robot.rightBack.setPower(-gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0)
        {
            robot.leftDrive.setPower(-gamepad1.right_trigger);
            robot.rightDrive.setPower(gamepad1.right_trigger);
            robot.leftBack.setPower(-gamepad1.right_trigger);
            robot.rightBack.setPower(gamepad1.right_trigger);
        }

        if(gamepad1.right_bumper)
            robot.intake.setPower(1);
        else if(gamepad1.left_bumper)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);

        if(gamepad2.y)
            robot.intake.setPower(1);
        else if(gamepad2.x)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);


        if(gamepad2.right_bumper) {
            robot.bottomSlider.setPower(-1);
            robot.topSlider.setPower(-1);
        }
        else if(gamepad2.left_bumper) {
            robot.bottomSlider.setPower(1);
            robot.topSlider.setPower(1);
        }
        else {
            robot.bottomSlider.setPower(0);
            robot.topSlider.setPower(0);
        }


        if(gamepad2.left_trigger > 0) {
            robot.leftShooter.setPower(-gamepad2.left_trigger/2);
            robot.rightShooter.setPower(gamepad2.left_trigger/2);
        }
        else if(gamepad2.right_trigger > 0) {
            robot.leftShooter.setPower(.48);
            robot.rightShooter.setPower(-.48);
        }
        else {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
        }
        if(gamepad2.dpad_left)
            robot.wobblyJoint.setPower(1);
        else if(gamepad2.dpad_right)
            robot.wobblyJoint.setPower(-1);
        if(gamepad2.a)
            robot.wobblyClaw.setPosition(1);
        else if(gamepad2.b)
            robot.wobblyClaw.setPosition(-1);
        }

}
}