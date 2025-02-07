package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(27);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(24);
  public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
  };

  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(16.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
  public static final double STEER_GEAR_RATIO = (50.0 / 14.0) * (60.0 / 10.0);

  // Pathplanner stuff
  public static final double WHEEL_COF = 1.2;

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(MODULE_TRANSLATIONS)
      .withRobotMass(Pounds.of(115))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getKrakenX60(1),
          DRIVE_GEAR_RATIO,
          STEER_GEAR_RATIO,
          Volts.of(0.1),
          Volts.of(0.1),
          Meters.of(WHEEL_RADIUS_METERS),
          KilogramSquareMeters.of(0.02),
          WHEEL_COF));


  public static final ModuleConstants[] MODULE_CONSTANTS;
  static {
    switch (Constants.getMode()) {
      case REAL -> {
        MODULE_CONSTANTS = new ModuleConstants[]{
            // Front Left
            ModuleConstants.builder()
                .driveId(3)
                .steerId(4)
                .encoderId(5)
                .encoderOffset(Rotation2d.fromDegrees(172.44).unaryMinus())
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Front Right
            ModuleConstants.builder()
                .driveId(6)
                .steerId(7)
                .encoderId(8)
                .encoderOffset(Rotation2d.fromDegrees(-101.60).unaryMinus().plus(Rotation2d.k180deg))
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Back Left
            ModuleConstants.builder()
                .driveId(12)
                .steerId(13)
                .encoderId(14)
                .encoderOffset(Rotation2d.fromDegrees(-4.57).unaryMinus())
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Back Right
            ModuleConstants.builder()
                .driveId(11)
                .steerId(10)
                .encoderId(12)
                .encoderOffset(Rotation2d.fromDegrees(60.20).unaryMinus().plus(Rotation2d.k180deg))
                .steerInverted(true)
                .turnInverted(false)
                .build()
        };
      }
      default -> {
        MODULE_CONSTANTS = new ModuleConstants[] {};
      }
    }
  }

  // per module config object
  @Builder
  public record ModuleConstants(
      int driveId,
      int steerId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean steerInverted
  ) {}
}
