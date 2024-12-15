package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MecanumBot_V1_Java", group = "Linear Opmode")
public class MecanumTeleOpJava extends LinearOpMode {

    // Declare OpMode members
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor towerArm = null;
    private DcMotor foreArm = null;
    private Servo claw = null;
    private CRServo intake = null;

    // Arm and Forearm target positions
    private static final int TOWER_ARM_POSITION_INIT = 0;
    private static final int TOWER_ARM_POSITION_HOVER_HIGH = 500;
    private static final int FORE_ARM_POSITION_INIT = 0;

    // Claw positions
    private static final double CLAW_OPEN_POSITION = 1.0;
    private static final double CLAW_CLOSED_POSITION = 0.0;

    // State machine for arm control
    private enum RobotState {
        INIT,
        HOVER_HIGH,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw state and button tracking
    private boolean clawOpen = true;
    private boolean lastClawToggle = false;

    // Target positions for arms
    private int targetTowerArm = TOWER_ARM_POSITION_INIT;
    private int targetForeArm = FORE_ARM_POSITION_INIT;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        towerArm = hardwareMap.get(DcMotor.class, "towerArm");
        foreArm = hardwareMap.get(DcMotor.class, "foreArm");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        towerArm.setDirection(DcMotor.Direction.FORWARD);
        foreArm.setDirection(DcMotor.Direction.FORWARD);

        // Reset and configure encoders
        towerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        foreArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize claw position
        claw.setPosition(CLAW_OPEN_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Mecanum drive controls
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.left_stick_x; // Left/right
            double rotate = gamepad1.right_stick_x; // Rotation

            // Calculate power for mecanum wheels
            double frontLeftPower = Range.clip(drive + strafe + rotate, -1.0, 1.0);
            double frontRightPower = Range.clip(drive - strafe - rotate, -1.0, 1.0);
            double backLeftPower = Range.clip(drive - strafe + rotate, -1.0, 1.0);
            double backRightPower = Range.clip(drive + strafe - rotate, -1.0, 1.0);

            // Set power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // State machine logic for arm control
            switch (currentState) {
                case INIT:
                    targetTowerArm = TOWER_ARM_POSITION_INIT;
                    targetForeArm = FORE_ARM_POSITION_INIT;
                    telemetry.addData("State", "INIT");
                    break;

                case HOVER_HIGH:
                    targetTowerArm = TOWER_ARM_POSITION_HOVER_HIGH;
                    telemetry.addData("State", "HOVER_HIGH");
                    break;

                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }

            // Handle state transitions
            if (gamepad1.a) {
                currentState = RobotState.HOVER_HIGH;
            } else if (gamepad1.b) {
                currentState = RobotState.INIT;
            } else if (gamepad1.dpad_up) {
                currentState = RobotState.MANUAL;
                targetTowerArm += 10;
            } else if (gamepad1.dpad_down) {
                currentState = RobotState.MANUAL;
                targetTowerArm -= 10;
            }

            // Update arm position
            towerArm.setTargetPosition(targetTowerArm);
            towerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            towerArm.setPower(1.0);

            // Claw toggle
             if (gamepad1.x){
                //move to position 0
                claw.setPosition(0);
    
            } else if (gamepad1.right_bumper) {
                //move to position 0.5
                claw.setPosition(0.5);

            } else if (gamepad1.y) {
                //move to position 1
                claw.setPosition(1);
                        }
            telemetry.addData("Servo Position", claw.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
            //if (gamepad1.right_bumper && !lastClawToggle) {
            //   clawOpen = !clawOpen;
            //     if (clawOpen) {
            //         claw.setPosition(CLAW_OPEN_POSITION);
            //     } else {
            //         claw.setPosition(CLAW_CLOSED_POSITION);
            //     }
            // }
            // lastClawToggle = gamepad1.right_bumper;

            // Intake controls
            // if (gamepad1.right_trigger > 0.1) {
            //     intake.setPower(1.0);
            // } else if (gamepad1.left_trigger > 0.1) {
            //     intake.setPower(-1.0);
            // } else {
            //     intake.setPower(0.0);
            // }

            // Telemetry feedback
            // telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            // telemetry.addData("Tower Arm Position", towerArm.getCurrentPosition());
            // telemetry.addData("Target Arm", targetTowerArm);
            // telemetry.update();
        }
    }
}
