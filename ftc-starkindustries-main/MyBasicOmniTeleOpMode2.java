// CODE FOR THE CONTROLLER PORTION

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "uncle code", group = "Linear OpMode")
public class MyBasicOmniTeleOpMode2 extends LinearOpMode {

    // Declare OpMode members for motors and servos.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontWheel = null; // Forward
    private DcMotor rightRearWheel = null; // Forward
    private DcMotor leftFrontWheel = null; // Backward
    private DcMotor leftRearWheel = null; // Backward

    private DcMotor towerArm = null;
    private DcMotor foreArm1 = null; // Declare REV Core Hex Motor here
    private DcMotor foreArm2 = null; // Declare REV Core Hex Motor here

    private Servo claw = null;

    @Override
    public void runOpMode() {

        // Initialize hardware variables with names matching the robot configuration.
        rightFrontWheel = hardwareMap.get(DcMotor.class, "frontRight"); // Forward
        rightRearWheel = hardwareMap.get(DcMotor.class, "backRight"); // Forward
        leftFrontWheel = hardwareMap.get(DcMotor.class, "frontLeft"); // Backward
        leftRearWheel = hardwareMap.get(DcMotor.class, "backLeft"); // Backward

        towerArm = hardwareMap.get(DcMotor.class, "towerArm");
        foreArm1 = hardwareMap.get(DcMotor.class, "foreArm1"); // Initialize the Core Hex Motor
        foreArm2 = hardwareMap.get(DcMotor.class, "foreArm2"); // Initialize the Core Hex Motor

        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions.
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);

        // Display status and update telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        claw.setPosition(0); //  Start with an Closed claw

        // Main loop
        while (opModeIsActive()) {

            // ------------ WHEELS -------------- //

            // AXIAL - Forward and Backward
            double axialPower = -gamepad1.left_stick_y;
            // LATERAL - Left and Right
            double lateralPower = gamepad1.left_stick_x;
            // YAW - Clockwise and Anti-clockwise
            double yawPower = 0.0;

            if (gamepad1.dpad_left) {
                yawPower = -0.1;
            }
            if (gamepad1.dpad_right) {
                yawPower = 0.1;
            }

            // Set motor power for drivetrain.
            rightFrontWheel.setPower(axialPower - lateralPower - yawPower);
            rightRearWheel.setPower(-axialPower + lateralPower - yawPower);

            leftFrontWheel.setPower(axialPower + lateralPower + yawPower);
            leftRearWheel.setPower(-axialPower - lateralPower + yawPower);


            // ------------ TOWER ARM -------------- //

            // Tower arm to Extend/Retract, on Y-axis of the Right joystick.
            double towerArmPower = -gamepad1.right_stick_y;
            towerArm.setPower(towerArmPower);


            // ------------ FORE ARM -------------- //

            //Fore arm to Extend on Left Trigger.
            if (gamepad1.left_trigger > 0) {
                foreArm1.setDirection(DcMotorSimple.Direction.FORWARD);
                foreArm2.setDirection(DcMotorSimple.Direction.FORWARD);
                foreArm1.setPower(gamepad1.left_trigger);
                foreArm2.setPower(gamepad1.left_trigger);
            }
            // Fore arm to Retract on Right Trigger.
            if (gamepad1.right_trigger > 0) {
                foreArm1.setDirection(DcMotorSimple.Direction.REVERSE);
                foreArm2.setDirection(DcMotorSimple.Direction.REVERSE);
                foreArm1.setPower(gamepad1.right_trigger);
                foreArm2.setPower(gamepad1.right_trigger);
            }

            // ------------ CLAW -------------- //

            double clawPower = gamepad1.right_stick_x;
            // Open claw
            if (clawPower > 0 && clawPower <= 1.0) {
                claw.setDirection(Servo.Direction.FORWARD);
            }
            // Open claw
            if (clawPower < 0 && clawPower >= -1.0)){
                claw.setDirection(Servo.Direction.REVERSE);
            }
            claw.setPosition(claw.getPosition() + clawPower);


            // Display telemetry data.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", "LF (%.2f), RF (%.2f), LB (%.2f), RB (%.2f)",
                    leftFrontWheel.getPower(), rightFrontWheel.getPower(), leftRearWheel.getPower(), rightRearWheel.getPower());

            telemetry.addData("Towerarm Power", "%.2f", towerArm.getPower());
            telemetry.addData("Towerarm Direction", "%s", towerArm.getDirection());
            telemetry.addData("Towerarm Current Position", "%.2f", towerArm.getCurrentPosition());
            telemetry.addData("Towerarm Target Position", "%.2f", towerArm.getTargetPosition());

            telemetry.addData("Fore Arm1 Power", "%.2f", foreArm1.getPower());
            telemetry.addData("Fore Arm1 Direction", "%s", foreArm1.getDirection());
            telemetry.addData("Fore Arm1 Current Position", "%.2f", foreArm1.getCurrentPosition());
            telemetry.addData("Fore Arm1 Target Position", "%.2f", foreArm1.getTargetPosition());

            telemetry.addData("Fore Arm2 Power", "%.2f", foreArm2.getPower());
            telemetry.addData("Fore Arm2 Direction", "%s", foreArm2.getDirection());
            telemetry.addData("Fore Arm2 Current Position", "%.2f", foreArm2.getCurrentPosition());
            telemetry.addData("Fore Arm2 Target Position", "%.2f", foreArm2.getTargetPosition());

            telemetry.addData("Claw Servo Position", "%.2f", claw.getPosition());
            telemetry.addData("Claw Servo Direction", "%s", claw.getDirection());
            telemetry.update();
        }
    }
}
