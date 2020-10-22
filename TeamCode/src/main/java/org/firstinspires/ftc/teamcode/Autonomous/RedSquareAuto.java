package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="redsquares", group = "basic")
public class RedSquareAuto extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){
            // left square
            rob.encoderMovement(0.5, 1, 10, 1000, Goal.movements.clawIn, rob.claw);
            rob.driveTrainEncoderMovement(0.2, 12, 5, 1000, Goal.movements.left);
            rob.driveTrainEncoderMovement(0.2, 80, 5, 50000, Goal.movements.forward);        }

            // low right square
        rob.encoderMovement(0.5, 1, 10, 1000, Goal.movements.clawIn, rob.claw);
        rob.driveTrainEncoderMovement(0.2, 6, 5, 1000, Goal.movements.right);
            rob.driveTrainEncoderMovement(0.2, 60, 5, 50000, Goal.movements.forward);

            // low right square
        rob.encoderMovement(0.5, 1, 10, 1000, Goal.movements.clawIn, rob.claw);
        rob.driveTrainEncoderMovement(0.2, 6, 5, 1000, Goal.movements.right);
            rob.driveTrainEncoderMovement(0.2, 90, 5, 50000, Goal.movements.forward);

    }
}
