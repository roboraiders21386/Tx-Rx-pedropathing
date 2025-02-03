package pedroPathing.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Arnav Doshi - 21386 Tx-Rx
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "SampleM3", group = "Examples")
public class SampleM3 extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private final Pose startPose = new Pose(0,0,0);
    private DcMotor Lift;
    private DcMotor Lift2;
    private Servo Sample;
    private Servo Rotation;
    private Servo Wrist;
    private Servo swap;
    //private DcMotor rig1; //right - motor port 0
    //private DcMotor rig2; //left - motor port 1
    private double dp = 1;
    private double intPow = 0.5;
    private double liftPow = 1;
    private int LIFT_INCREMENT = 10;
    private int a = 100;
    private int b = -100;
    private int tcnt = 0;
    double rotCor = 0.0175;
    private int leftcnt = 0;
    private int sampcnt = 0;
    private int swapcnt = 0;
    private int score = 0;
    private int pick = 0;
    private double rotPos = 0.17, rotPick = 0.34, rotDelta = 0.05, rotWait = 0.3;
    private double wristScore = 0, wristPick = 1, wristSpecial = (wristScore*3+wristPick)/4.0;
    private double closeClaw = 0, openClaw = 0.3, wristSpecPick = 0.3;
    private double rotSpec = 0.1;
    private DcMotor rig1;
    private DcMotor rig2;
    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);
        pathTimer = new Timer();
        rig1 = hardwareMap.get(DcMotor.class, "rig1");
        rig2 = hardwareMap.get(DcMotor.class, "rig2");
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Sample = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        rig1 = hardwareMap.get(DcMotor.class, "rig1");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        rig1.setDirection(DcMotorSimple.Direction.REVERSE);
        //rig1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rig2.setDirection(DcMotorSimple.Direction.FORWARD);
        //rig2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Wrist = hardwareMap.get(Servo.class, "wrist");
        swap = hardwareMap.get(Servo.class, "switch");
        /*double ServoPosition;
        double ServoSpeed;
        int currentPos;
        ServoPosition = 1;
        ServoSpeed = 1;*/
        int clawUse = 0;
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {

        follower.startTeleopDrive();
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);

        Lift2.setTargetPosition(0);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(liftPow);

        Rotation.setPosition(0.1+rotCor);
        Sample.setPosition(0);
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Lift 1 position:", Lift.getCurrentPosition());
        telemetry.addData("Lift 2 position:", Lift2.getCurrentPosition());
        telemetry.addData("Rotation position:", Rotation.getPosition());
        telemetry.addData("Sample claw position:", Sample.getPosition());
        telemetry.addData("Wrist position:", Wrist.getPosition());
        telemetry.addData("Rig1 position", rig1.getCurrentPosition());
        //StartIntake
        if (!gamepad1.start && gamepad1.right_bumper) {
            if (sampcnt%2==0) Sample.setPosition(openClaw);
            else Sample.setPosition(closeClaw);
            sampcnt++;
            sleep(500);
        }
        if (!gamepad1.start && gamepad1.left_bumper) {
            if (swapcnt%2==0) swap.setPosition(0.9);
            else swap.setPosition(0.4);
            swapcnt++;
            sleep(500);
            telemetry.addData("Swap Position: ", swap.getPosition());
        }
        if (gamepad1.left_trigger>0) {
            Wrist.setPosition(0);
        }
        if (gamepad1.right_trigger>0) {
            Wrist.setPosition(1);
        }
        if (gamepad1.options && gamepad1.dpad_down) { //low specimen
            //score specimen low
            Rotation.setPosition(0.24+rotCor);//0.21
            telemetry.addData("Rotate", "set to 0.24");
            // sleep(300);
            //Intake.setPosition(0.3);
        }
        if (!gamepad1.options && gamepad1.dpad_up) {
            //Grab Position
            Wrist.setPosition(wristPick);
            //Sample.setPosition(0.3);
            Rotation.setPosition(rotPick+0.01);//0.33);
            sleep(300);
            Sample.setPosition(closeClaw);
            Rotation.setPosition(rotWait);
            telemetry.addData("Rotate", "set to 1");
        }
        else if (!gamepad1.options && gamepad1.dpad_down) {
            //All the way backwards
            Rotation.setPosition(0+rotCor);//0.25
            Wrist.setPosition(0.15);
            telemetry.addData("Rotate", "set to 0");
            // sleep(300);
            //Intake.setPosition(0.3);
        }

        if (!gamepad1.options && gamepad1.dpad_right) {
            if (pick%2==0) {
                Lift.setTargetPosition(0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(0);
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                Rotation.setPosition(rotSpec);
                Wrist.setPosition(wristSpecPick);
                Sample.setPosition(openClaw);
                telemetry.addData("Rotate", "set to 0.04");
            } else {
                Sample.setPosition(closeClaw);
            }
            pick++;
            sleep(250);
            //Pick Up Specimen from Human Player

        } else if (!gamepad1.options && gamepad1.dpad_left) {
            //Midway position
            if (leftcnt%2==0) {
                Rotation.setPosition(rotWait);
                Wrist.setPosition(wristPick);
                telemetry.addData("Rotate", "set to 0.3");
            } else {
                Rotation.setPosition(rotWait+0.01);
            }
            leftcnt++;
            sleep(250);
        }

        if (gamepad1.options && gamepad1.dpad_left) {
            //Decremtal
            Rotation.setPosition(Rotation.getPosition()-rotDelta);
            sleep(500);
        }
        if (gamepad1.options && gamepad1.dpad_right) {
            //Incremental
            Rotation.setPosition(Rotation.getPosition()+rotDelta);
            sleep(500);
        }
        if (!gamepad1.share && gamepad1.cross) {
            //Align Straight and go down
            Rotation.setPosition(0.2);
            Wrist.setPosition(wristPick);
            Lift.setTargetPosition(0);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(liftPow);
            Lift2.setTargetPosition(0);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setPower(liftPow);
            //while (Lift.isBusy() && Lift2.isBusy()) {
            telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
            telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
            telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
            telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
            telemetry.update();
            //}
            sleep(500);

        }
        if (gamepad1.share && gamepad1.cross) {
            Wrist.setPosition(0);
            //Low Basket Sample
            Rotation.setPosition(rotPos);
            Lift.setTargetPosition(1300);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(liftPow);
            Lift2.setTargetPosition(1300);
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setPower(liftPow);
            //while (Lift.isBusy() && Lift2.isBusy()) {
            telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
            telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
            telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
            telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
            telemetry.update();
            //}
            Rotation.setPosition(0.13);
            sleep(500);

        }
        if (!gamepad1.share && gamepad1.triangle) {
            Wrist.setPosition((wristPick+wristScore)/2);
            Rotation.setPosition(rotPos);
            Lift.setTargetPosition(3610);//To be updated, Belt is loose
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(liftPow);
            Lift2.setTargetPosition(3610);//To be updated, Belt is loose
            Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift2.setPower(liftPow);
            //while (Lift.isBusy() && Lift2.isBusy()) {
            telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
            telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
            telemetry.addData("Current Position Lift 2:", Lift2.getCurrentPosition());
            telemetry.addData("Target Position Lift 2:", Lift2.getTargetPosition());
            telemetry.update();
            //}
            Rotation.setPosition(0.1+rotCor);
            sleep(500);
        }

        if (gamepad1.square){
            Lift.setTargetPosition(Lift.getCurrentPosition()-100);
            Lift2.setTargetPosition(Lift2.getCurrentPosition()-100);
            Lift.setPower(liftPow);
            Lift2.setPower(liftPow);
            telemetry.addData("Current Pos Lift:", Lift.getCurrentPosition());
            telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
        }

        if (gamepad1.share && gamepad1.square){
            //get ready to rig
            //rig2.setPower(1);
            //rig1.setPower(1);
            //sleep(5000);
            rig1.setTargetPosition(0);
            rig2.setTargetPosition(0);
            rig1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rig2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rig2.setPower(1);
            rig1.setPower(1);
        }


        if (gamepad1.share && gamepad1.circle){
            //get ready to rig
            rig1.setTargetPosition(3100);
            rig1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rig2.setTargetPosition(3100);
            rig2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rig1.setPower(1);
            rig2.setPower(1);
        }
        if (gamepad1.circle){
            Lift.setTargetPosition(Lift.getCurrentPosition()+100);
            Lift2.setTargetPosition(Lift2.getCurrentPosition()+100);
            Lift.setPower(liftPow);
            Lift2.setPower(liftPow);
            telemetry.addData("Current Pos Lift: ", Lift.getCurrentPosition());
            telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
        }
        if  (gamepad1.share && gamepad1.triangle) {
            if (score%2==0) {
                Wrist.setPosition(wristPick);
                Rotation.setPosition(rotPos+0.005);
                swap.setPosition(0.4);
                Lift.setTargetPosition(350);
                Lift2.setTargetPosition(350);
                Lift.setPower(liftPow);
                Lift2.setPower(liftPow);
                telemetry.addData("Current Pos Lift: ", Lift.getCurrentPosition());
                telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
            } else {
                Rotation.setPosition(rotPos+0.02);
                sleep(500);
                Lift.setTargetPosition(1200);
                Lift2.setTargetPosition(1200);
                Lift.setPower(liftPow);
                Lift2.setPower(liftPow);
                telemetry.addData("Current Pos Lift: ", Lift.getCurrentPosition());
                telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
                sleep(1000);
                Sample.setPosition(openClaw);
            }
            score++;
            sleep(200);
        }
        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
    public void sleep(int x) {
        pathTimer.resetTimer();
        double y = x/1000.0;
        while (pathTimer.getElapsedTimeSeconds()<y) {
            y = y;
        }
    }
}