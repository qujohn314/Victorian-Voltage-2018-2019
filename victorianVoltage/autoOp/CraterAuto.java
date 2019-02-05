
package org.firstinspires.ftc.teamcode.opModes.victorianVoltage.autoOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

/**
 * @author johnson_891609
 */
@Autonomous(name = "CraterAuto")
public class CraterAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive,lift,flip,spool;
    private Servo phoneMount,marker;//,bucketServo,trapDoor;
    private CRServo intakeServo;
    private BNO055IMU gyro;
    private final double GEAR_RATIO = 1, WHEEL_DIAMETER = 4, TICKS_PER_REV_DRIVE = 560,LIFT_RATIO=.3333,LIFT_DIAMETER=0.819;
    private final double DC = TICKS_PER_REV_DRIVE / (Math.PI * WHEEL_DIAMETER * GEAR_RATIO);
    private final double TICKS_PER_REV_LIFT=1680,maxheight = 22.75,minheight = 15.875,STRAFE_RATIO = 1;
    private final double LC = (TICKS_PER_REV_LIFT/(Math.PI*LIFT_DIAMETER*LIFT_RATIO));
    private MineralSensor camera;

    private static Orientation angles;
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
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
        lift.setDirection(DcMotor.Direction.FORWARD);

        flip = (DcMotorEx)hardwareMap.dcMotor.get("flip");

        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setDirection(DcMotor.Direction.REVERSE);



        spool = (DcMotorEx)hardwareMap.dcMotor.get("spool");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setDirection(DcMotor.Direction.FORWARD);

        phoneMount = hardwareMap.servo.get("phoneMount");
        phoneMount.setDirection(Servo.Direction.FORWARD);
        phoneMount.setPosition(0);

        marker = hardwareMap.servo.get("marker");
        marker.setDirection(Servo.Direction.FORWARD);
        marker.setPosition(0);

        intakeServo = hardwareMap.crservo.get("intakeServo");
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeServo.setPower(0);

        camera = new MineralSensor(this);

        waitForStart();
        phoneMount.setPosition(.5);
        try {
            disconnect();
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
            flip.setPower(0);
            move(5,1);
            strafeLeft(3);
            while(opModeIsActive() && camera.mineral == MineralSensor.Mineral.NONE) {

                mineralSense();
                heartbeat();

            }

            switch (camera.mineral.getName()) {

                case "Center":
                    phoneMount.setPosition(0);
                    strafeRight(3);
                    move(18, 0.8);
                    //intakeMotor.setPower(0);

                    move(-8,0.8);
                    //  move(-45,0.8);
                    turn(77,"left");


                    move(43,0.8);
                    turn(10,"left");
                    strafeLeft(35);
                    move(44,1);
                    deposit();
                    move(-45,0.75);
                    strafeRight(4);
                    turn(150,"left");
                    marker.setPosition(0.4);
                    strafeRight(20);

                    move(27,1);
                    crater();


                    break;
                case "Left":
                    phoneMount.setPosition(0);
                    move(7, 1);
                    //intakeMotor.setPower(0);
                    strafeRight(19);
                    move(12,0.8);
                    move(-9,0.8);
                    //  move(-45,0.8);
                    turn(76,"left");


                    move(30,1);
                    turn(10,"left");
                    strafeLeft(35);
                    move(44,1);
                    deposit();
                    move(-47,1);
                    strafeLeft(12);
                    turn(150,"left");
                    marker.setPosition(0.4);
                    strafeRight(20);

                    move(25,1);

                    crater();

                    break;
                case "Right":
                    phoneMount.setPosition(0);
                    move(11, 1);
                    //intakeMotor.setPower(0);
                    strafeLeft(12);
                    move(11,1);
                    move(-9,1);
                    //  move(-45,0.8);
                    turn(79,"left");


                    move(54,1);
                    turn(15,"left");
                    strafeLeft(40);
                    move(43,1);
                    deposit();
                    move(-45,1);
                    strafeRight(4);
                    turn(150,"left");
                    marker.setPosition(0.7);
                    strafeRight(20);

                    move(25,1);
                    crater();

                    break;

            }

        }catch (InterruptedException e){}

    }
    public void disconnect()throws InterruptedException{
        int ticks = (int)(6.2*LC);
        telemetry.addData("Total Ticks: ", ticks);
        telemetry.update();
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setTargetPosition(-ticks);
        lift.setPower(1);
        while (lift.isBusy() && opModeIsActive()) {
            heartbeat();
            telemetry.addData("Total Ticks: ", ticks);
            telemetry.addData("Current Ticks: ", lift.getCurrentPosition());
            telemetry.update();
        }
        lift.setPower(0);

    }
    public void turn(int degree,String dir)throws InterruptedException{
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
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int deg = 0;
        if (dir.equals("right"))
            deg = degree;
        else if (dir.equals("left"))
            deg = degree;
        telemetry.addData("desired angle", deg);
        telemetry.update();
        if (dir.equals("left")) {
            while ((int)Math.abs(angles.firstAngle)  < deg && opModeIsActive()) {
                leftFrontDrive.setPower(-.5);
                leftRearDrive.setPower(-.5);
                rightFrontDrive.setPower(.5);
                rightRearDrive.setPower(.5);

                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }
        else if (dir.equals("right")) {
            while (deg != 0 && (int)Math.abs(angles.firstAngle)  < deg && opModeIsActive()) {

                leftFrontDrive.setPower(.5);
                leftRearDrive.setPower(.5);
                rightFrontDrive.setPower(-.5);
                rightRearDrive.setPower(-.5);

                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
    public void strafeRight(double distance) throws InterruptedException {
        int ticks = (int) (distance * DC*1.061947);
        int ticksadj = (int) (distance* DC * 1.061947*1.188119);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(-ticksadj);
        leftFrontDrive.setPower(-.4);

        leftRearDrive.setTargetPosition(ticks);
        leftRearDrive.setPower(.4);

        rightFrontDrive.setTargetPosition(ticksadj);
        rightFrontDrive.setPower(.4);

        rightRearDrive.setTargetPosition(-ticks);
        rightRearDrive.setPower(-.4);
        while (opModeIsActive() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {
            heartbeat();
        }
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
    }

    public void strafeLeft(double distance) throws InterruptedException {
        int ticks = (int) (distance * DC*1.061947);
        int ticksadj = (int) (distance* DC * 1.061947*1.188119);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(ticksadj);
        leftFrontDrive.setPower(.4);

        leftRearDrive.setTargetPosition(-ticks);
        leftRearDrive.setPower(-.4);

        rightFrontDrive.setTargetPosition(-ticksadj);
        rightFrontDrive.setPower(-.4);

        rightRearDrive.setTargetPosition(ticks);
        rightRearDrive.setPower(.4);
        while (opModeIsActive() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {
            heartbeat();
        }
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
    }
    private void deposit()throws InterruptedException{
        runtime.reset();
        double tim = runtime.time();
        marker.setPosition(1);
        while(tim + 0.5 > runtime.time() && opModeIsActive()) {
            marker.setPosition(1);
            heartbeat();
        }
        move(-6,0.75);
        marker.setPosition(0.3);
    }

    private void waitForS(double t) throws InterruptedException{
        runtime.reset();
        double tim = runtime.time();
        while(tim + t > runtime.time() && opModeIsActive()) {
            heartbeat();
        }
    }

    private void crater()throws InterruptedException{
        runtime.reset();
        double tim = runtime.time();
        marker.setPosition(0.8);
        while(tim + 1 > runtime.time() && opModeIsActive()) {
            flip.setPower(0.3);
            heartbeat();
        }
        flip.setPower(0);
    }
    public void move(double distance, double power) throws InterruptedException {
        int ticks = (int) (distance * DC);
        if(distance<0) {
            //  ticks *= -1;
            power *= -1;
        }
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(ticks);
        leftFrontDrive.setPower(power);

        leftRearDrive.setTargetPosition(ticks);
        leftRearDrive.setPower(power);

        rightFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setPower(power);

        rightRearDrive.setTargetPosition(ticks);
        rightRearDrive.setPower(power);
        while (opModeIsActive() && leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {
            heartbeat();
        }
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
    }
    private void mineralSense() throws InterruptedException{
        ArrayList<Integer> setting = new ArrayList<Integer>();
        camera.startTFOD();
        runtime.reset();
        double tim = runtime.time();
        while(camera.mineral == MineralSensor.Mineral.NONE && opModeIsActive()){
            // telemetry.addData("ORIENTATION","FINDING" + camera.mineral.getName());
            camera.update();
            // telemetry.update();
            if(camera.mineral != MineralSensor.Mineral.NONE){
                while(setting.size()<=250 && opModeIsActive()){
                    heartbeat();
                    if(camera.mineral == MineralSensor.Mineral.CENTER)
                        setting.add(0);
                    else if(camera.mineral == MineralSensor.Mineral.RIGHT)
                        setting.add(1);
                    else if(camera.mineral == MineralSensor.Mineral.LEFT)
                        setting.add(2);
                    heartbeat();

                }
            }
            heartbeat();
            if(tim + 6 < runtime.time() && camera.mineral == MineralSensor.Mineral.NONE) {
                setting.add(0);
                heartbeat();
                break;
            }
        } camera.stopTFOD();

        int left = 0;
        int right = 0;
        int center = 0;
        for(Integer x : setting)
            switch(x) {
                case 0:
                    center++;
                    break;
                case 1:
                    right++;
                    break;
                case 2:
                    left++;
                    break;
            }
        if(center > left && center > right)
            camera.mineral = MineralSensor.Mineral.CENTER;
        else if(left > center && left > right)
            camera.mineral = MineralSensor.Mineral.LEFT;
        else
            camera.mineral = MineralSensor.Mineral.RIGHT;

        telemetry.addData("ORIENTATION",""+camera.mineral.getName());
        telemetry.update();
    }
    private void heartbeat() throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}

