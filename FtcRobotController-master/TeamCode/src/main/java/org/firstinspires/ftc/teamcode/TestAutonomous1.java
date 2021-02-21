package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name="Test Autonomous", group="BionicBot")
public class TestAutonomous1 extends LinearOpMode
{
    TestHardwareBionicbot         robot   = new TestHardwareBionicbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE           = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (CIRCUMFERENCE);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    BNO055IMU imu;

    Orientation angles;
    Orientation newAngles;
    @Override
    public void runOpMode()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);


        waitForStart();

        double firstD = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        LeftDiagonal(1,1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("Roll", angles.secondAngle);
        telemetry.addData("Pitch",angles.thirdAngle);
        telemetry.update();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        GyroCorrect(firstD,imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


//        robot.BackColorSensor.enableLed(true);
//        robot.FrontColorSensor.enableLed(true);




//        telemetry.addData("Color Number of Left Color Sensor: ", robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));

    }
    public void DriveForwardDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveBackwardDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBackward(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnLeftDistance(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnLeft(speed);


        while( robot.rightDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.leftDrive.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnRightDistance(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnRight(speed);


        while(robot.leftDrive.isBusy()  && robot.leftBack.isBusy() && robot.rightDrive.isBusy()  && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafeLeftDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);


        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        StrafeLeft(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftDrive.isBusy()) {

        }

        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafeRightDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);



        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        StrafeRight(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        while(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.rightDrive.isBusy())
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void RightDiagonal(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition());
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition());

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void LeftDiagonal(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition());
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition());
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveForward(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(-power);
    }
    public void DriveBackward(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
    }
    public void TurnLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(-power);
    }
    public void TurnRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(power);
    }
    public void StrafeLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);
    }
    public void StrafeRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
    }

    public void GyroCorrect(double degree1, double degree2){
        if(degree1 < degree2){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > degree1 + 0.5){
                robot.leftDrive.setPower(-.1);
                robot.rightDrive.setPower(.1);
                robot.leftBack.setPower(-.1);
                robot.rightBack.setPower(.1);
            }
        }
        else if(degree1 > degree2){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < degree1 - 0.5){
                robot.leftDrive.setPower(.1);
                robot.rightDrive.setPower(-.1);
                robot.leftBack.setPower(.1);
                robot.rightBack.setPower(-.1);
            }
        }
    }

    public void GyroFlip(double degree){
        if(degree > -180 && degree < -160){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > degree - 4){
                robot.leftDrive.setPower(.1);
                robot.rightDrive.setPower(-.1);
                robot.leftBack.setPower(-.1);
                robot.rightBack.setPower(.1);
            }
        }
        else if(degree < 180 && degree > 160){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < degree + 4){
                robot.leftDrive.setPower(-.1);
                robot.rightDrive.setPower(.1);
                robot.leftBack.setPower(.1);
                robot.rightBack.setPower(-.1);
            }
        }



    }
    public void Gyro180()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -175) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -120){
                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(1);
                robot.rightBack.setPower(1);
                robot.leftBack.setPower(-1);
            }
            else{
                robot.leftDrive.setPower(-0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBack.setPower(0.2);
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

    public void StopDriving()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
    public void DriveForward(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void LeftDiagonal(double power, int sleepTime)
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DriveBackward(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void TurnLeft(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void TurnRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void StrafeRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void StrafeLeft(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DownLeft(double power, int sleepTime)
    {
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftDrive.setPower(power);
        robot.rightBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DownRight(double power, int sleepTime)
    {
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(power);
        robot.leftDrive.setPower(0);
        robot.rightBack.setPower(0);
        sleep(sleepTime);
        StopDriving();
    }
    public void UpLeft(double power, int sleepTime)
    {
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void UpRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }

}