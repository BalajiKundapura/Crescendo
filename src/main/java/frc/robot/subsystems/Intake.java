package frc.robot.subsystems;

import static java.lang.Math.abs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends ProfiledPIDSubsystem {
    private DigitalInput beamBreak = new DigitalInput(8);
    private CANSparkMax intakePull = new CANSparkMax(10, MotorType.kBrushless);
    private CANSparkMax intakePull2 = new CANSparkMax(16, MotorType.kBrushless);

    private CANSparkMax intakeToggle = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax intakeToggle2 = new CANSparkMax(17, MotorType.kBrushless);

    private AbsoluteEncoder toggleEncoder = intakeToggle2.getAbsoluteEncoder();

    public enum intakePosition {
        up(14),
        raised(35), // 30 // 37 // 46
        none(13),
        down(130),
        custom(14);

        public double angle;

        private intakePosition(double value) {
            this.angle = value;
        }
    }

    intakePosition currentTarget = intakePosition.up;

    public Intake() {
        super(new ProfiledPIDController(.14, 0, 0, new Constraints(1000, 1000)));
        SmartDashboard.putData("Intake Controller", getController());
        intakePull.restoreFactoryDefaults();
        intakePull2.restoreFactoryDefaults();       


        intakeToggle.restoreFactoryDefaults();
        intakeToggle2.restoreFactoryDefaults();
        intakeToggle2.follow(intakeToggle, true);
        intakePull.follow(intakePull2);
        getController().setTolerance(5);
        getController().enableContinuousInput(0, 360);
        intakeToggle.setIdleMode(IdleMode.kCoast);
        intakeToggle2.setIdleMode(IdleMode.kCoast);
        
        intakeToggle.setInverted(true);
        
        toggleEncoder.setPositionConversionFactor(360); // 135.0/28.0
        toggleEncoder.setInverted(true);
        intakeToggle.burnFlash();
        intakeToggle2.burnFlash();
        setGoal(intakePosition.up.angle);
    }

    public double GetIntakeAngle() {
        return toggleEncoder.getPosition();
    }

    public intakePosition GetIntakePosition() {
        if (GetIntakeAngle() > 120 && GetIntakeAngle() < 300) {
            return intakePosition.down;
        } else if (GetIntakeAngle() < 15 || GetIntakeAngle() > 300) {
            return intakePosition.up;
        } else
            return intakePosition.none;
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    public Command intakeNote(CommandXboxController controller) {
        return either(sequence(
                SetIntakePosition(intakePosition.down),
                runOnce(() -> {
                    setVoltage(6);
                }),
                waitUntil(() -> {
                    return hasNote();
                }),
                runOnce(() -> {
                    stopIntake();
                }),
                runOnce(() -> {
                    controller.getHID().setRumble(RumbleType.kBothRumble, 1);
                }),
                SetIntakePosition(intakePosition.up))
                .finallyDo(() -> {
                    controller.getHID().setRumble(RumbleType.kBothRumble, 0);
                    stopIntake();
                }), none(), ()->{return !hasNote();});
    }

    public Command SetIntakePosition(intakePosition position) {
        return runOnce(() -> {
            currentTarget = position;
            setGoal(position.angle);
        }).andThen(waitUntil(() -> {
            return getController().atGoal();
        }));
    }

    public void stopIntake() {
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        intakePull2.setVoltage(-voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Angle", toggleEncoder.getPosition());
        SmartDashboard.putBoolean("Intake Has Note", hasNote());
        SmartDashboard.putString("Arm Rotation Position", currentTarget.name());
        SmartDashboard.putBoolean("Arm At Goal", getController().atGoal());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        intakeToggle.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
        return toggleEncoder.getPosition();
    }

    public void setBrake(boolean b) {
        
        intakeToggle.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);

        intakeToggle2.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public Command rotateToCustomAngle(double d) {
        intakePosition.custom.angle = d;
        return SetIntakePosition(intakePosition.custom);
    }
}
