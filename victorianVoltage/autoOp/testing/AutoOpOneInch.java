package org.firstinspires.ftc.teamcode.opModes.victorianVoltage.autoOp.testing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name = "AutoTestOne",group = "Auto")
/**
 * @author johnson_891609
 */
//MEASUREMENTS IN INCHES


public class AutoOpOneInch extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx lift, intakeMotor,flip,spool;
    private Servo phoneMount,marker, intakeServo;
    private BNO055IMU gyro;
    private final double GEAR_RATIO = 1, WHEEL_DIAMETER = 4, TICKS_PER_REV_DRIVE = 560,LIFT_RATIO=.5;
    private final double DC = TICKS_PER_REV_DRIVE * GEAR_RATIO / (Math.PI * WHEEL_DIAMETER);
    private final double SPOOL_DIAMETER = 1.296,TICKS_PER_REV_LIFT=1680,maxheight = 22.75,minheight = 15.875,STRAFE_RATIO = 1;
    private final double LC = TICKS_PER_REV_LIFT/(Math.PI*SPOOL_DIAMETER*LIFT_RATIO);
    private TensorTestVert camera;

    //                  0-LFD | 1-LRD | 2-RFD | 3-RRD
    private DcMotorEx[] driveTrain = new DcMotorEx[4];

    private static Orientation angles;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
        for (int i = 0; i < driveTrain.length; i++) {
            if (i == 0) {
                driveTrain[i] = (DcMotorEx) hardwareMap.dcMotor.get("drvFrontLeft");
                driveTrain[i].setDirection(DcMotor.Direction.FORWARD);
            } else if (i == 1) {
                driveTrain[i] = (DcMotorEx) hardwareMap.dcMotor.get("drvRearLeft");
                driveTrain[i].setDirection(DcMotor.Direction.FORWARD);
            } else if (i == 2) {
                driveTrain[i] = (DcMotorEx) hardwareMap.dcMotor.get("drvFrontRight");
                driveTrain[i].setDirection(DcMotor.Direction.REVERSE);
            } else {
                driveTrain[i] = (DcMotorEx) hardwareMap.dcMotor.get("drvRearRight");
                driveTrain[i].setDirection(DcMotor.Direction.REVERSE);
            }
            driveTrain[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveTrain[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveTrain[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        flip = (DcMotorEx) hardwareMap.dcMotor.get("flip");
        flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setDirection(DcMotor.Direction.REVERSE);


        spool = (DcMotorEx)hardwareMap.dcMotor.get("spool");
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        phoneMount = hardwareMap.servo.get("phoneMount");
        phoneMount.setDirection(Servo.Direction.FORWARD);
        phoneMount.setPosition(0.5);

        marker = hardwareMap.servo.get("marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);

        intakeServo = hardwareMap.servo.get("intakeServo");
        intakeServo.setDirection(Servo.Direction.FORWARD);
        intakeServo.setPosition(0);

        camera = new TensorTestVert(this);

        waitForStart();

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro = hardwareMap.get(BNO055IMU.class, "gyro");
            gyro.initialize(parameters);
            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        waitForStart();
        try {
            strafe(18);
        }catch (InterruptedException e){}

    }

    public void move(double distance, double power) throws InterruptedException {
        int ticks = (int) (distance * DC);
        if(distance<0) {
            ticks *= -1;
            power *= -1;
        }

        for(DcMotor motor: driveTrain){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            motor.setPower(power);
        }


        while (driveTrain[0].isBusy() && driveTrain[1].isBusy() && driveTrain[2].isBusy() && driveTrain[3].isBusy() && opModeIsActive()) {
            heartbeat();
        }
        stopTrain();

    }

    private void strafe(double distance) throws InterruptedException{
        int ticks = 0;


        driveTrain[0].setDirection(DcMotor.Direction.FORWARD);
        driveTrain[1].setDirection(DcMotor.Direction.REVERSE);

        driveTrain[3].setDirection(DcMotor.Direction.FORWARD);
        driveTrain[2].setDirection(DcMotor.Direction.REVERSE);


        for(DcMotor motor : driveTrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        ticks = (int)((Math.abs(distance)*DC));
        double power = 1;

        for(DcMotor motor : driveTrain) {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
        }


        while (driveTrain[3].isBusy() || driveTrain[2].isBusy() || driveTrain[1].isBusy() || driveTrain[0].isBusy()) {
            telemetry.addData("Target Ticks: ", ""+ticks);
           for(DcMotor motor : driveTrain) {
               telemetry.addData("Motor Ticks: ", "" + motor.getCurrentPosition());

               telemetry.update();
           }
            heartbeat();
        }
        stopTrain();
    }

    private void stopTrain(){
        driveTrain[0].setPower(0);
        driveTrain[1].setPower(0);
        driveTrain[2].setPower(0);
        driveTrain[3].setPower(0);
    }

    private void heartbeat() throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}
