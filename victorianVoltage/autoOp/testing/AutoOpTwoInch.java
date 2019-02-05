package org.firstinspires.ftc.teamcode.opModes.victorianVoltage.autoOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
/**
 * @author johnson_891609
 */
@Autonomous(name = "AutoTestTwo",group = "Auto")

//MEASUREMENTS IN INCHES


public class AutoOpTwoInch extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive,lift, intakeMotor;
    private Servo phoneMount,marker,flip;
    private CRServo intakeServo;
    private final double GEAR_RATIO = 1, WHEEL_DIAMETER = 4, TICKS_PER_REV_DRIVE = 560;
    private final double DC = (TICKS_PER_REV_DRIVE * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER);
    private final double SPOOL_DIAMETER = 1.296,TICKS_PER_REV_LIFT=1680,maxheight = 22.75,minheight = 15.875;
    private final double LC = TICKS_PER_REV_LIFT/(Math.PI*SPOOL_DIAMETER);
    private static double curheight=15.875;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
        leftFrontDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontLeft");
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvRearLeft");
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvFrontRight");
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive = (DcMotorEx)hardwareMap.dcMotor.get("drvRearRight");
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        lift = (DcMotorEx)hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor = (DcMotorEx)hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        phoneMount = hardwareMap.servo.get("phoneMount");
        phoneMount.setDirection(Servo.Direction.FORWARD);
        phoneMount.setPosition(0);
        marker = hardwareMap.servo.get("marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);
        flip = hardwareMap.servo.get("flip");
        flip.setDirection(Servo.Direction.FORWARD);
        flip.setPosition(0);
        intakeServo = hardwareMap.crservo.get("intakeServo");
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(0);
        waitForStart();
        try {
            strafe(2);
        }catch (InterruptedException e){}

    }

    private void strafe(double distance) throws InterruptedException{
        int ticks = 0;


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);

        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);



        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticks = (int)((Math.abs(distance)*DC));
        double power = 0.5;

        rightFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setPower(power);

        leftFrontDrive.setTargetPosition(ticks);
        leftFrontDrive.setPower(power);

        rightRearDrive.setTargetPosition(ticks);
        rightRearDrive.setPower(power);

        leftRearDrive.setTargetPosition(ticks);
        rightFrontDrive.setPower(power);

        while (rightFrontDrive.isBusy() || leftFrontDrive.isBusy() || rightRearDrive.isBusy() || leftRearDrive.isBusy()) {
            telemetry.addData("Target Ticks: ", ""+ticks);
            telemetry.addData("LF Motor Ticks: ", ""+leftFrontDrive.getCurrentPosition());
            telemetry.addData("LR Motor Ticks: ", ""+leftRearDrive.getCurrentPosition());
            telemetry.addData("RF Motor Ticks: ", ""+rightFrontDrive.getCurrentPosition());
            telemetry.addData("RR Motor Ticks: ", ""+rightRearDrive.getCurrentPosition());
            heartbeat();

        }
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    private void heartbeat() throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}
