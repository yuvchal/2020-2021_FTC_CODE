package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

@Autonomous (name="TestColorSensing", group="BionicBot")

public class ColorSensingClass extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    public ModernRoboticsI2cColorSensor LeftColorSensor  = null;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE           = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (CIRCUMFERENCE);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;





    @Override
    public void runOpMode()  {

        LeftColorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("Left_Color");


        waitForStart();

        LeftColorSensor.enableLed(true);

        while(opModeIsActive())
        {
            telemetry.addData("Color Number", LeftColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));
            telemetry.update();
        }

    }}
