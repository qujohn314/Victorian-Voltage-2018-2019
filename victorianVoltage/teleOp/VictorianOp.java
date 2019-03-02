package org.firstinspires.ftc.teamcode.opModes.victorianVoltage.teleOp;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * @Created by warham_905444 on 10/3/2017.
 * @Edited by johnson_891609 on 2/01/2019
 */


@TeleOp(name="VictorianOp",group="TeleOp")
public class VictorianOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive,lift,flip,spool;
    private Servo phoneMount,marker/*,bucketServo*/,trapDoor;
    private CRServo intakeServo;
    private boolean precision,direction,foron,backon,trap,bucket,flipped;
    private boolean canTogglePrecision,canToggleDirection,canToggleIntake,canToggleTrap;
    private double intakepower = 0,TICKS_PER_REV=1680;
    private final double LC = ((2.25*TICKS_PER_REV)/360);
    private BNO055IMU gyro;
    private static Orientation angles;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontLeft");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        leftRearDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvRearLeft");
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontRight");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        rightRearDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvRearRight");
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        lift = (DcMotorEx)hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        flip = (DcMotorEx)hardwareMap.dcMotor.get("flip");
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setDirection(DcMotor.Direction.REVERSE);
        //when toggle make reverse
        spool = (DcMotorEx)hardwareMap.dcMotor.get("spool");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setDirection(DcMotor.Direction.FORWARD);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        phoneMount = hardwareMap.servo.get("phoneMount");
        phoneMount.setDirection(Servo.Direction.FORWARD);
        phoneMount.setPosition(0);

        marker = hardwareMap.servo.get("marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);

        trapDoor = hardwareMap.servo.get("trapDoor");
        trapDoor.setDirection(Servo.Direction.FORWARD);
        trapDoor.setPosition(0.7);

//        bucketServo = hardwareMap.servo.get("bucketServo");
//        bucketServo.setDirection(Servo.Direction.REVERSE);
//        bucketServo.setPosition(0);

        intakeServo = hardwareMap.crservo.get("intakeServo");
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(0);
        precision = false;
        direction = true;
        foron=false;
        backon=false;
        trap =false;
        bucket = true;
        flipped= true;



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time " + runtime.toString());
        double liftPower;

        // Precision mode toggled by pressing A
        if(gamepad1.right_stick_button && canTogglePrecision){
            precision = !precision;
            canTogglePrecision = false;
        }
        else if(!gamepad1.right_stick_button) {
            canTogglePrecision = true;
        }

        if(gamepad1.y && canToggleDirection){
            direction = !direction;
            canToggleDirection = false;
        }
        else if(!gamepad1.y) {
            canToggleDirection = true;
        }

        double y = gamepad2.left_stick_y;
        final double flipPow = Range.clip(y, -1.0, 1.0);
        flip.setPower(.4*flipPow);

        double z = gamepad2.right_stick_y;
        final double spoolpower = Range.clip(z, -1.0, 1.0);
        spool.setPower(spoolpower*1);

        if(((gamepad2.b&&!backon)||(gamepad2.b&&foron))&&canToggleIntake){
            intakepower=-.45;
            backon=true;
            foron=false;
            canToggleIntake=false;
        }
        else if(gamepad2.b&&canToggleIntake){
            intakepower=0;
            backon=false;
            foron=false;
            canToggleIntake=false;
        }
        else if(!gamepad2.a&&!gamepad2.b){
            canToggleIntake=true;
        }

        if(((gamepad2.a&&!foron)||((gamepad2.a&&backon)))&&canToggleIntake) {
            intakepower = .8;
            foron=true;
            backon=false;
            canToggleIntake=false;
        }
        else if(gamepad2.a&&canToggleIntake) {
            intakepower = 0;
            foron=false;
            backon=false;
            canToggleIntake=false;
        }
        else if(!gamepad2.a&&!gamepad2.b){
            canToggleIntake=true;
        }


        if((gamepad2.x&&!trap&&canToggleTrap)) {
            trapDoor.setPosition(0.73);
            trap = true;
            canToggleTrap=false;
        }
        else if(gamepad2.x&&canToggleTrap) {
            trapDoor.setPosition(.42);
            trap = false;
            canToggleTrap=false;
        }
        else if(!gamepad2.x) {
            canToggleTrap = true;
        }

        if(gamepad1.right_bumper){
            liftPower=1;
        }
        else if(gamepad1.left_bumper){
            liftPower = -1;
        }
        else{
            liftPower= 0;
        }
        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = 0.5*gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) + rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x* Math.cos(powerAngle) - rightX, -1.0, 1.0);



        //(Code Below) If (?) precision than 0.1 else (:) 1
        leftFrontDrive.setPower(leftFrontPower * (precision ? 0.25 : 1));
        leftRearDrive.setPower(leftRearPower * (precision ? 0.25 : 1));
        rightFrontDrive.setPower(rightFrontPower * (precision ? 0.25 : 1));
        rightRearDrive.setPower(rightRearPower * (precision ? 0.25 : 1));
        intakeServo.setPower(intakepower);
        lift.setPower(liftPower);
        // Send DS wheel power values.
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftRearPower, rightRearPower);
        telemetry.addData("Arm Position: ", flip.getCurrentPosition());
        telemetry.addData("angle", angles.firstAngle);
        telemetry.addData("angle", angles.secondAngle);
        telemetry.addData("angle", angles.thirdAngle);

        telemetry.update();
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}



