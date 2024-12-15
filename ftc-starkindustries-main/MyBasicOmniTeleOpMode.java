// CODE FOR THE CONTROLLER PORTION

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Enhanced: 4-Motor Omni Drive with Dual Forearms and Claw", group="Linear OpMode")
public class MyBasicOmniTeleOpMode extends LinearOpMode {

    // Declare OpMode members for motors and servos.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontWheel = null; // Forward
    private DcMotor rightRearWheel = null; // Forward
    private DcMotor leftFrontWheel = null; // Backward
    private DcMotor leftRearWheel = null; // Backward

    private DcMotor towerArm = null;
    private DcMotor foreArm = null; // Declare REV Core Hex Motor here

    private Servo intake = null;
    private Servo claw = null;

    private boolean lastBump = false;


    @Override
    public void runOpMode() {

        // Initialize hardware variables with names matching the robot configuration.
        rightFrontWheel = hardwareMap.get(DcMotor.class, "frontRight"); // Forward
        rightRearWheel = hardwareMap.get(DcMotor.class, "backRight"); // Forward
        leftFrontWheel = hardwareMap.get(DcMotor.class, "frontLeft"); // Backward
        leftRearWheel = hardwareMap.get(DcMotor.class, "backLeft"); // Backward

        towerArm = hardwareMap.get(DcMotor.class, "towerArm");
        foreArm = hardwareMap.get(DcMotor.class, "foreArm"); // Initialize the Core Hex Motor

        intake = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions.
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);

        towerArm.setDirection(DcMotor.Direction.FORWARD);
        foreArm.setDirection(DcMotor.Direction.FORWARD); // Set Core Hex Motor direction

        // Display status and update telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main loop
        while (opModeIsActive()) {
            // AXIAL - Forward and Backward
            double axialPower = -gamepad1.left_stick_y;
            // LATERAL - Left and Right
            double lateralPower = gamepad1.left_stick_x;
            // YAW - Clockwise and Anti-clockwise
            double yawPower = gamepad1.right_stick_x;
            
            // Set motor power for drivetrain.
            rightFrontWheel.setPower(axialPower - lateralPower - yawPower);
            rightRearWheel.setPower(axialPower + lateralPower - yawPower);
            leftFrontWheel.setPower(axialPower + lateralPower + yawPower);
            leftRearWheel.setPower(axialPower - lateralPower + yawPower);
//
//            // Tower arm to move Forward/Backward, on Y-axis of the Right joystick.
//            double towerArmPower = -gamepad1.right_stick_y;
//            towerArm.setPower(towerArmPower);
//
//            // Fore arm to move Forward/Backward on Right/Left Triggers.
//            if(gamepad1.right_trigger>0.0){
//                foreArm.setPower(gamepad1.right_trigger);
//            }
//            if(gamepad1.left_trigger>0.0){
//                foreArm.setPower(-gamepad1.left_trigger);
//            }
//
//            // PRESETS for Arms
//            if (gamepad1.dpad_up) {
//                towerArm.setPower(0.5);
//                foreArm.setPower(0.5);
//            } else if (gamepad1.dpad_down) {
//                towerArm.setPower(-0.5);
//                foreArm.setPower(-0.5);
//            }
//
//            // Intake using Right/Left Bumpers.
//            while (gamepad1.right_bumper) {
//                intake.setPosition(0.1); // Rotate servo forward
//            }
//            while (gamepad1.left_bumper) {
//                intake.setPosition(-0.1); // Rotate servo backward
//            }
//
//            // Claw Open/Close with X button
//            claw.setPosition(1.0); //  Start with an Open claw
//            if (gamepad1.x) { //while press and hold
//                claw.setPosition(0.2); // Close claw
//            } else { // when released
//                claw.setPosition(0.8); // Open claw
//            }
//
//            lastBump = gamepad1.x;

            // Display telemetry data.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)",
                    leftFrontWheel.getPower(), rightFrontWheel.getPower(), leftRearWheel.getPower(), rightRearWheel.getPower());
            telemetry.addData("Towerarm Power", "%.2f", towerArm.getPower());
            telemetry.addData("Fore Arm Power", "%.2f", foreArm.getPower()); // Telemetry for Core Hex Motor
            telemetry.addData("Intake Servo Position", "%.2f", intake.getPosition());
            telemetry.addData("Claw Servo Position", "%.2f", claw.getPosition());
            telemetry.update();
        }
    }
}
