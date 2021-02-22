package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 * Motor channel:  Raising the claw:         "shoulder"
 * Motor channel:  Raising the joint:        "elbow"
 * Motor channel:  Pull mechanism:           "shaft"
 *
 *
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareBionicbot
{
/* Public OpMode members. */
public DcMotor leftBack = null;
public DcMotor rightBack = null;
public DcMotor leftDrive   = null;
public DcMotor rightDrive  = null;
public DcMotor intake  = null;
public DcMotor leftShooter  = null;
public DcMotor rightShooter  = null;
public DcMotor wobblyJoint = null;
public Servo wobblyClaw = null;
public CRServo topSlider = null;
public CRServo bottomSlider = null;
public Servo distanceServo = null;
public DistanceSensor distanceSensor = null;
public Servo LServo = null;
public CRServo rotServo = null;
public DcMotor shootSlider = null;


/*public DcMotor pincher = null;
public DcMotor lifter = null;
public DcMotor lifter1 = null;
public DcMotor straight = null;
public Servo Rightglock = null;
public Servo Leftglock = null;*/
//public ModernRoboticsI2cColorSensor LeftColorSensor  = null;
//public ModernRoboticsI2cColorSensor BackColorSensor = null;
//public ModernRoboticsI2cColorSensor FrontColorSensor = null;


public final static double ARM_HOME = 0.0;
public final static double MIN_ARM_RANGE = 0.0;
public final static double MAX_ARM_RANGE = 1.0;

//public DcMotor dustbin = null;
public DcMotor[] motors = new DcMotor[4];
//public Servo binBlock = null;
//public AnalogInput pixy = null;
//public GyroUnshafter gyro;
//public ModernRoboticsI2cGyro gyro = null;

HardwareMap hwMap  = null;
private ElapsedTime period  = new ElapsedTime();

/* Constructor */
public HardwareBionicbot() {
}

/* Initialize standard Hardware interfaces */
public void init(HardwareMap ahwMap) {
    hwMap = ahwMap;
    //Color_Sensor = hwMap.get(ColorSensor.class, "sensor_color");
//    IntegratingGyroscope gyro;
//    ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    leftDrive  = hwMap.get(DcMotor.class, "left_drive");
    rightDrive = hwMap.get(DcMotor.class, "right_drive");
    leftBack = hwMap.get(DcMotor.class, "left_back");
    rightBack = hwMap.get(DcMotor.class, "right_back");
    intake = hwMap.get(DcMotor.class, "intake");
    leftShooter = hwMap.get(DcMotor.class, "left_shooter");
    wobblyJoint = hwMap.get(DcMotor.class, "wobbly_joint");
    wobblyClaw = hwMap.get(Servo.class, "wobbly_claw");
    distanceSensor = hwMap.get(DistanceSensor.class,"distance_sensor");
    LServo = hwMap.get(Servo.class, "launch_servo");
    rotServo = hwMap.get(CRServo.class, "rotServo");
    shootSlider = hwMap.get(DcMotor.class, "shootSlider");

//    modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
//    gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
    /*lifter = hwMap.get(DcMotor.class,"lifter");
    pincher = hwMap.get(DcMotor.class, "pincher");
    lifter1 = hwMap.get(DcMotor.class,"lifter1");
    straight = hwMap.get(DcMotor.class,"straight");
    Rightglock = hwMap.get(Servo.class, "Largeglock");
    Leftglock = hwMap.get(Servo.class, "Smallglock");*/
//    LeftColorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class,"LeftColor");
//    BackColorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "BackColor");
//    FrontColorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "FrontColor");







    //shaft1 = hwMap.get(DcMotor.class, "shaft1");
    //shaft2 = hwMap.get(DcMotor.class, "shaft2");
    //rake = hwMap.get(DcMotor.class, "rake");
    //pixy = hwMap.get(AnalogInput.class, "pixy");

    //grabberL = hwMap.get(Servo.class, "gripL");
    //grabberR = hwMap.get(Servo.class, "gripR");
    //tail = hwMap.get(Servo.class, "tail");
    //gyro = new GyroUnshafter(hwMap.get(ModernRoboticsI2cGyro.class, "gyro"));
  //  gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");


    motors[0] = rightDrive;
    motors[1] = rightBack;
    motors[2] = leftDrive;
    motors[3] = leftBack;


    //shaft1.setPower(0);
    //shaft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    //shaft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //rake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
}