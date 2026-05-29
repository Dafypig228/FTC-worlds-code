package org.firstinspires.ftc.teamcode.controlllers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
private DcMotorEx intake;

	public Intake(HardwareMap hardwareMap) {

		intake = hardwareMap.get(DcMotorEx.class, "intake");
		intake.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public Action intake() { return new IntakeStart(); }

	public Action outtake() { return new OuttakeStart(); }

	public Action stop() { return new Stop(); }


	public class IntakeStart implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			intake.setPower(1);
			return false;
		}
	}
	public void intakeVoid(){
		intake.setPower(1);
	}

	public class OuttakeStart implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			intake.setPower(-1);
			return false;
		}
	}
	public void OuttakeStart(){
		intake.setPower(-1);
	}

	public class Stop implements Action {
		@Override
		public boolean run(@NonNull TelemetryPacket packet) {
			intake.setPower(0);
			return false;
		}
	}
	public void Stop() {
		intake.setPower(0);
	}

}
