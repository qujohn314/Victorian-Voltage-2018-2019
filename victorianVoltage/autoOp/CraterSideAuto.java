
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
@Autonomous(name = "CraterSideAuto")
public class CraterSideAuto extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive,lift,flip,spool;
    private Servo phoneMount,marker, trapDoor;//,bucketServo,trapDoor;
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

        trapDoor = hardwareMap.servo.get("trapDoor");
        trapDoor.setDirection(Servo.Direction.FORWARD);


        camera = new MineralSensor(this);


        waitForStart();

        try {
            disconnect();
            runtime.reset();

            double tim = runtime.time();
            while(tim +  0.5> runtime.time() && opModeIsActive()) {
                flip.setPower(0.1);
                telemetry.addData("spool power: ",spool.getPower());
                telemetry.addData("spool position: ",spool.getCurrentPosition());
                telemetry.update();
                heartbeat();
            }
            phoneMount.setPosition(.5);


            flip.setPower(0);

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



            move(4,0.5);


            while(opModeIsActive() && camera.mineral == MineralSensor.Mineral.NONE) {
                mineralSense();
                heartbeat();

            }
            runtime.reset();
            tim = runtime.time();
            phoneMount.setPosition(0);
            while(tim + 1 > runtime.time() && opModeIsActive()) {
                flip.setPower(-1);
                telemetry.addData("spool power: ",spool.getPower());
                telemetry.addData("spool position: ",spool.getCurrentPosition());
                telemetry.update();
                heartbeat();
            }
            runtime.reset();
            tim = runtime.time();

            phoneMount.setPosition(0);

            flip.setPower(0);
            switch (camera.mineral.getName()) {

                case "Center":
                    phoneMount.setPosition(0);
                    move(19, 0.6);
                    //intakeMotor.setPower(0);

                    move(-9,0.6);
                    //  move(-45,0.8);
                    turn(90,"left");


                    move(40,1);
                    turn(131,"left");
                    strafeLeft(25);
                    move(47,1);
                    deposit();
                    move(-45,1);
                    turn(-32,"left");
                    marker.setPosition(0.7);

                    move(15,1);
                    crater();


                    break;
                case "Left":
                    phoneMount.setPosition(0);
                    turn(40,"left");
                    //intakeMotor.setPower(0);
                    move(23,1);

                    //  move(-45,0.8);
                    move(-9,0.7);
                    turn(90,"left");


                    move(30,1);
                    turn(131,"left");
                    strafeLeft(25);
                    move(47,1);
                    deposit();
                    move(-47,1);
                    turn(-32,"left");
                    marker.setPosition(0.7);

                    move(15,1);

                    crater();

                    break;
                case "Right":
                    phoneMount.setPosition(0);
                    turn(-38,"left");
                    //intakeMotor.setPower(0);
                    move(23,1);
                    move(-9,0.7);
                    //  move(-45,0.8);
                    turn(90,"left");


                    move(50,1);
                    turn(131,"left");
                    strafeLeft(25);
                    move(47,1);
                    deposit();
                    move(-45,1);
                    turn(-32,"left");
                    marker.setPosition(0.7);

                    move(25,1);
                    crater();

                    break;

            }

        }catch (InterruptedException e){}

    }
    public void disconnect()throws InterruptedException{
        int ticks = (int)(5.6*LC);
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
    public void turn(int degree,String dir)throws InterruptedException {
        if(degree > 180)
            dir = dir.equals("right") ? "left" : "right";


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double deg = degree;
        //Time control for velocity calculations
        double currentT = runtime.time();
        double deltaT = 0;


        double oldDeg = Math.abs(angles.firstAngle);
        double deltaDeg = 0;


        telemetry.addData("desired angle", deg);
        telemetry.update();


        double LFDmotorPower = 0;
        double LRDmotorPower = 0;
        double RFDmotorPower = 0;
        double RRDmotorPower = 0;


        double LFDCorrection = 10;
        double LRDCorrection = 10;
        double RFDCorrection = 10;
        double RRDCorrection = 10;

        double tAV = 0;

        boolean over = false;
        int direction = dir.equals("left") ? 1 : -1;

        boolean firstTrial = true;
        double firstCorrection = 0;

        double angleError = deg > 100 ? 6 : 2.5;

        Pid turnControlLFD = new Pid(0.38, 0, 0.06);
        Pid turnControlLRD = new Pid(0.38, 0, 0.06);
        Pid turnControlRFD = new Pid(0.38, 0, 0.06);
        Pid turnControlRRD = new Pid(0.38, 0, 0.06);
        runtime.reset();

        double angleDistance = angles.firstAngle >= 0 && deg < 0 ? Math.abs((angles.firstAngle - deg - 360)) : Math.abs((angles.firstAngle - deg ));;

        while (angleDistance  > angleError  && opModeIsActive()) {

            deltaT = runtime.time() - currentT;
            currentT = runtime.time();

            angleDistance = angles.firstAngle >= 0 && deg < 0 ? Math.abs((angles.firstAngle - deg - 360)) : Math.abs((angles.firstAngle - deg ));


            if (angleDistance > 60)
                tAV = 700;
            else if (angleDistance > 20)
                tAV = 300;
            else {
                tAV = 50 * ((angleDistance) / 15);
            }

            LFDCorrection = turnControlLFD.controlOutput((over ? tAV : -tAV) * direction, leftFrontDrive.getVelocity(AngleUnit.DEGREES), deltaT);
            LRDCorrection = turnControlLRD.controlOutput((over ? tAV : -tAV) * direction, leftRearDrive.getVelocity(AngleUnit.DEGREES), deltaT);
            RFDCorrection = turnControlRFD.controlOutput((over ? -tAV : tAV) * direction, rightFrontDrive.getVelocity(AngleUnit.DEGREES), deltaT);
            RRDCorrection = turnControlRRD.controlOutput((over ? -tAV : tAV) * direction, rightRearDrive.getVelocity(AngleUnit.DEGREES), deltaT);

            LFDmotorPower += LFDCorrection;
            LRDmotorPower += LRDCorrection;
            RFDmotorPower += RFDCorrection;
            RRDmotorPower += RRDCorrection;


            if (angles.firstAngle < deg)
                over = false;
            else
                over = true;





            leftFrontDrive.setVelocity((LFDmotorPower), AngleUnit.DEGREES);
            leftRearDrive.setVelocity(LRDmotorPower, AngleUnit.DEGREES);

            rightFrontDrive.setVelocity(RFDmotorPower, AngleUnit.DEGREES);
            rightRearDrive.setVelocity(RRDmotorPower, AngleUnit.DEGREES);

            angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("angle", angles.firstAngle);
            telemetry.addData("Target Angle", deg);
                    /*
                    telemetry.addData("LFD Velocity", LFDVelocity);
                    telemetry.addData("LRD Velocity", LRDVelocity);
                    telemetry.addData("RFDMotor Velocity", RFDVelocity);
                    telemetry.addData("RRDMotor Velocity", RRDVelocity);
                    telemetry.addData("Target Angle Velocity", targetAngleVelocity);
                    telemetry.addData("Target Velocity", targetVelocity);
                    telemetry.addData("LFDPower", LFDmotorPower);

    */

            telemetry.addData("LFDCorrection: ", LFDCorrection);
            telemetry.addData("LRDCorrection: ", LRDCorrection);
            telemetry.addData("RFDCorrection: ", RFDCorrection);
            telemetry.addData("RRDCorrection: ", RRDCorrection);

            telemetry.addData("First Correction: ", firstCorrection);

            double percentError = (Math.abs(deg - Math.abs(angles.firstAngle)) / deg) * 100;
            telemetry.addData("LFD VELOCITY: ", leftFrontDrive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("LRD VELOCITY: ", leftRearDrive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("RFD VELOCITY: ", rightFrontDrive.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("RRD VELOCITY: ", rightRearDrive.getVelocity(AngleUnit.DEGREES));


            telemetry.addData("Percent Error: ", percentError + "%");
            telemetry.update();


            firstTrial = false;
            heartbeat();

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
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        double tim = runtime.time();
        marker.setPosition(0.8);
        move(-3,1);

        trapDoor.setPosition(0.73);
        while(tim + 2 > runtime.time() && opModeIsActive()) {
            flip.setPower(0.6);

            intakeServo.setPower(1);


            heartbeat();
        }
        flip.setPower(0);
        intakeServo.setPower(1);
        spool.setTargetPosition(1000);
        spool.setPower(1);
        runtime.reset();
        tim = runtime.time();

        spool.setPower(1);
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
                while(setting.size()<=150 && opModeIsActive()){
                    heartbeat();
                    if(camera.mineral == MineralSensor.Mineral.CENTER)
                        setting.add(0);
                    else if(camera.mineral == MineralSensor.Mineral.RIGHT)
                        setting.add(1);
                    else if(camera.mineral == MineralSensor.Mineral.LEFT)
                        setting.add(2);


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

