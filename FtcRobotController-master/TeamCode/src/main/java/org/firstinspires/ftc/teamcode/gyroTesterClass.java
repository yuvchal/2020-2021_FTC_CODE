package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;
import android.media.AudioRecord;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "gyroTesterClass", group = "BionicBot")

public class gyroTesterClass extends LinearOpMode
{
    HardwareBionicbot robot = new HardwareBionicbot();
    BNO055IMU imu;

    Orientation angles;
    Orientation newAngles;

    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Roll", angles.secondAngle);
            telemetry.addData("Pitch",angles.thirdAngle);
            telemetry.update();
            GyroLeft();
            GyroRight();
            sleep(1000);
        }

    }
    public void AutoAligner() {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
            robot.leftDrive.setPower(.4);
            robot.rightDrive.setPower(-.4);
            robot.rightBack.setPower(-.4);
            robot.leftBack.setPower(.4);
        }
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
            robot.leftDrive.setPower(-.4);
            robot.rightDrive.setPower(.4);
            robot.rightBack.setPower(.4);
            robot.leftBack.setPower(-.4);
        }
    }



    public void GyroLeft()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 90) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 25){
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
                robot.rightBack.setPower(1);
                robot.leftBack.setPower(1);
            }
            else{
                robot.leftDrive.setPower(0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBack.setPower(0.2);
                robot.leftBack.setPower(0.2);
            }
            telemetry.addData("Heading", newAngles.firstAngle);
            telemetry.addData("Roll", newAngles.secondAngle);
            telemetry.addData("Pitch",newAngles.thirdAngle);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
    }
    public void GyroRight()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -90) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -25){
                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(-1);
                robot.rightBack.setPower(-1);
                robot.leftBack.setPower(-1);
            }
            else{
                robot.leftDrive.setPower(-0.2);
                robot.rightDrive.setPower(-0.2);
                robot.rightBack.setPower(-0.2);
                robot.leftBack.setPower(-0.2);
            }
            telemetry.addData("Heading", newAngles.firstAngle);
            telemetry.addData("Roll", newAngles.secondAngle);
            telemetry.addData("Pitch",newAngles.thirdAngle);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
    }

}
