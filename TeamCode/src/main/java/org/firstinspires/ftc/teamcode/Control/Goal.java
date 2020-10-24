package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_COREHEXMOTOR_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.COUNTS_PER_GOBUILDA435RPM_INCH;
import static org.firstinspires.ftc.teamcode.Control.Constants.claws;
import static org.firstinspires.ftc.teamcode.Control.Constants.imuS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorBRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFLS;
import static org.firstinspires.ftc.teamcode.Control.Constants.motorFRS;
import static org.firstinspires.ftc.teamcode.Control.Constants.pincher;

public class Goal {


    public Goal(HardwareMap hardwareMap, ElapsedTime runtime, Central central, setupType... setup) throws InterruptedException {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.central = central;

        StringBuilder i = new StringBuilder();

        for (setupType type: setup) {
            switch (type) {
                case teleop:
                    setupDrivetrain();
                    setupClaw();
                    break;
            }

            i.append(type.name()).append(" ");

        }
        central.telemetry.addLine(i.toString());
        central.telemetry.update();

    }

    // important non-confdiguration field
    public ElapsedTime runtime;     //set in constructor to the runtime of running class
    public Central central;
    public HardwareMap hardwareMap;

    public boolean target = false;
    public boolean moving1 = false;
    public boolean moving2 = false;
    public boolean moving3 = false;
    public boolean stop = false;
    public boolean straight = false;
    public int x = 0;
    public int y = 0;
    public int blockNumber = 0;

    public int[] wheelAdjust = {1, 1, 1, 1};

    public static double speedAdjust = 20.0 / 41.0;
    public static double yToXRatio = 1.25;

    public void setWheelAdjust(int fr, int fl, int br, int bl) {
        wheelAdjust[0] = fr;
        wheelAdjust[1] = fl;
        wheelAdjust[2] = br;
        wheelAdjust[3] = bl;
    }
    //----specfic non-configuration fields
    //none rnh


    // Vuforia Variables
    public final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public final boolean PHONE_IS_PORTRAIT = false;
    public final String VUFORIA_KEY =
             " AYzLd0v/////AAABmR035tu9m07+uuZ6k86JLR0c/MC84MmTzTQa5z2QOC45RUpRTBISgipZ2Aop4XzRFFIvrLEpsop5eEBl5yu5tJxK6jHbMppJyWH8lQbvjz4PAK+swG4ALuz2M2MdFXWl7Xh67s/XfIFSq1UJpX0DgwmZnoDCYHmx/MnFbyxvpWIMLZziaJqledMpZtmH11l1/AS0oH+mrzWQLB57w1Ur0FRdhpxcrZS9KG09u6I6vCUc8EqkHqG7T2Zm4QdnytYWpVBBu17iRNhmsd3Ok3w8Pn22blBYRo6dZZ8oscyQS1ZtilM1YT49ORQHc8mu/BMWh06LxdstWctSiGiBV0+Wn3Zk++xQ750c64lg3QLjNkXc";
    public final float mmPerInch        = 25.4f;
    public final float mmTargetHeight   = (6) * mmPerInch;
    public final float stoneZ = 2.00f * mmPerInch;
    public final float bridgeZ = 6.42f * mmPerInch;
    public final float bridgeY = 23 * mmPerInch;
    public final float bridgeX = 5.18f * mmPerInch;
    public final float bridgeRotY = 59;
    public final float bridgeRotZ = 180;
    public final float halfField = 72 * mmPerInch;
    public final float quadField  = 36 * mmPerInch;

    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public WebcamName webcamName = null;

    //----------------CONFIGURATION FIELDS--------------------
    public DcMotor[] drivetrain;   //set in motorDriveMode() for drivetrain movement functions

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public DcMotor claw;

    public Servo pinch;

    public BNO055IMUImpl imu;


    public double StrafetoTotalPower = 2.0/3.0;

    //----       IMU        ----

    public BNO055IMUImpl.Parameters imuparameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    public static boolean isnotstopped;
    public float initorient;

    public void setupIMU() throws InterruptedException{
        imuparameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        imuparameters.loggingTag = "imu"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = hardwareMap.get(BNO055IMUImpl.class, imuS);
        imu.initialize(imuparameters);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        central.telemetry.addData("IMU status", imu.getSystemStatus());
        central.telemetry.update();


    }

    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }

    public void setupClaw() throws InterruptedException {
        claw = motor(claws, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        pinch = servo(pincher, Servo.Direction.FORWARD, 0, 1, 0);

        motorDriveMode(EncoderMode.ON, claw);
    }




    //-----------------------HARDWARE SETUP FUNCTIONS---------------------------------------
    public DcMotor motor(String name, DcMotor.Direction directionm, DcMotor.ZeroPowerBehavior zeroPowerBehavior) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setPower(0);
        return motor;
    }

    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(startSpeed);
        return servo;
    }
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        central.telemetry.addData("Beacon Red Value: ", sensor.red());
        central.telemetry.update();

        return sensor;
    }
    public ModernRoboticsI2cRangeSensor ultrasonicSensor(String name) throws InterruptedException {

        return hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }

    public ModernRoboticsI2cColorSensor MRColor(String name) throws InterruptedException{
        return hardwareMap.get(ModernRoboticsI2cColorSensor.class, name);

    }

    public void encoder(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

    }

    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                central.idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA435RPM_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();

                for (int i = 0; i < drivetrain.length; i++) {
                    DcMotor motor = drivetrain[i];
                    if (!motor.isBusy() && signs[i] != 0) {
                        x = false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_GOBUILDA435RPM_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }
    public void encodeCoreHexMovement(double speed, double distance, double timeoutS, long waitAfter, movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (central.opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * wheelAdjust[x] * distance * COUNTS_PER_COREHEXMOTOR_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (central.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                central.idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            central.sleep(waitAfter);


        }
    }

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void driveTrainMovement(double... speed) throws InterruptedException{

        for (int i = 0; i < drivetrain.length; i++) {
            drivetrain[i].setPower(speed[i]);
        }
    }
    public void driveTrainTimeMovement(double speed, movements movement, long duration, long waitAfter) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        stopDrivetrain();
        central.sleep(waitAfter);
    }

    public void anyMovement(double speed, movements movement, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
    }
    public void anyMovementTime(double speed, movements movement, long duration, DcMotor... motors) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: motors){
            int x = Arrays.asList(motors).indexOf(motor);
            motor.setPower(signs[x] * wheelAdjust[x]* speed);

        }
        central.sleep(duration);
        for (DcMotor motor: motors){
            motor.setPower(0);

        }
    }
    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }

    public void powerMotors(double speed, long time, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
        central.sleep(time);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    // IMU Movements
    public void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = getDirection();

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", (calculateDifferenceBetweenAngles(end, getDirection())));
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }
        central.sleep(5000);

        while (calculateDifferenceBetweenAngles(getDirection(), end) < -0.25 && central.opModeIsActive()) {
            driveTrainMovement(0.1, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", calculateDifferenceBetweenAngles(end, getDirection()));
            central.telemetry.update();

        }
        stopDrivetrain();
        central.sleep(5000);

        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", calculateDifferenceBetweenAngles(end, getDirection()));
        central.telemetry.update();
        central.sleep(5000);
    }
    public void absturn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{

        central.telemetry.addData("IMU State: ", imu.getSystemStatus());
        central.telemetry.update();

        double start = 0;

        double end = (start + ((direction == turnside.cw) ? target : -target) + 360) % 360;

        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (InterruptedException e) {
            isnotstopped = false;
        }

        while (((calculateDifferenceBetweenAngles(getDirection(), end) > 1 && turnside.cw == direction) || (calculateDifferenceBetweenAngles(getDirection(), end) < -1 && turnside.ccw == direction)) && central.opModeIsActive() ) {
            central.telemetry.addLine("First Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Difference: ", end - getDirection());
            central.telemetry.update();
        }
        try {
            stopDrivetrain();
        } catch (InterruptedException e) {
        }

        while (calculateDifferenceBetweenAngles(end, getDirection()) > 1 && central.opModeIsActive()){
            driveTrainMovement(0.05, (direction == turnside.cw) ? movements.ccw : movements.cw);
            central.telemetry.addLine("Correctional Try ");
            central.telemetry.addData("IMU Inital: ", start);
            central.telemetry.addData("IMU Final Projection: ", end);
            central.telemetry.addData("IMU Orient: ", getDirection());
            central.telemetry.addData("IMU Diffnce: ", end - getDirection());
            central.telemetry.update();
        }
        stopDrivetrain();
        central.telemetry.addLine("Completed");
        central.telemetry.addData("IMU Inital: ", start);
        central.telemetry.addData("IMU Final Projection: ", end);
        central.telemetry.addData("IMU Orient: ", getDirection());
        central.telemetry.addData("IMU Diffnce: ", end - getDirection());
        central.telemetry.update();
    }

    public double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle) // negative is secondAngle ccw relative to firstAngle
    {
        double difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return -difference;
    }

    public double getDirection(){
        return (this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-initorient+720)%360;
    }


    public enum EncoderMode{
        ON, OFF;
    }
    public enum setupType{
        autonomous, teleop, endgame, drive, camera, claw, tfod, bSystem, foundation, yellow, encoder, intake, ultrasoinc, imu;
    }



    //-------------------SET FUNCTIONS--------------------------------
    public void setCentral(Central central) {
        this.central = central;
    }
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    //-------------------CHOICE ENUMS-------------------------
    public enum movements
    {
        // FR FL BR BL
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        forward(1, -1, 1, -1),
        backward(-1, 1, -1, 1),
        br(0, -1, 1, 0),
        bl(1, 0, 0, -1),
        tl(0, 1, -1, 0),
        tr(-1, 0, 0, 1),
        cw(1, 1, 1, 1),
        ccw(-1, -1, -1, -1),
        cwback(-1, -1, 0, 0),
        ccwback(1, 1, 0, 0),
        cwfront(0, 0, -1, -1),
        ccwfront(0, 0, 1, 1),
        linearUp(1),
        linearDown(-1),
        clawOut(-1),
        clawIn(1);



        private final double[] directions;

        movements(double... signs) {
            this.directions = signs;
        }

        public double[] getDirections() {
            return directions;
        }
    }


    public enum turnside {
        ccw, cw
    }

    public static double[] anyDirection(double speed, double angleDegrees) {
    double theta = Math.toRadians(angleDegrees);
    double beta = Math.atan(yToXRatio);

    double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
    double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

    double[] retval = {v1, v2};
    return retval;
    }

    public static double[] anyDirectionRadians(double speed, double angleRadians) {
    double theta = angleRadians;
    double beta = Math.atan(yToXRatio);

    double v1 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) + speed * Math.cos(theta) / Math.cos(beta));
    double v2 = speedAdjust * (speed * Math.sin(theta) / Math.sin(beta) - speed * Math.cos(theta) / Math.cos(beta));

    double[] retval = {v1, v2};
    return retval;
    }

    public void driveTrainMovementAngle(double speed, double angle) {

    double[] speeds = anyDirection(speed, angle);
    motorFR.setPower(movements.forward.directions[0] * speeds[0]);
    motorFL.setPower(movements.forward.directions[1] * speeds[1]);
    motorBR.setPower(movements.forward.directions[2] * speeds[1]);
    motorBL.setPower(movements.forward.directions[3] * speeds[0]);

    }

    public void driveTrainMovementAngleRadians(double speed, double angle) {

    double[] speeds = anyDirectionRadians(speed, angle);
    motorFR.setPower(movements.forward.directions[0] * speeds[0]);
    motorFL.setPower(movements.forward.directions[1] * speeds[1]);
    motorBR.setPower(movements.forward.directions[2] * speeds[1]);
    motorBL.setPower(movements.forward.directions[3] * speeds[0]);

    }

    public enum axis {
        front, center, back
    }


}
