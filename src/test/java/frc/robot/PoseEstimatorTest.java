package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.Test;

public class PoseEstimatorTest {

    // 0.307975 is 12.125 in inches
    public static final @NotNull Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(0.307975, 0.307975);
    public static final @NotNull Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.307975, 0.307975);
    public static final @NotNull Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.307975, -0.307975);
    public static final @NotNull Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-0.307975, -0.307975);
    public static final @NotNull Translation2d @NotNull [] SWERVE_MODULE_LOCATIONS = {
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION
    };

    @NotNull public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            SWERVE_MODULE_LOCATIONS
    );

    @Test
    public void testPoseEstimator() {
        var estimator = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                new Rotation2d(),
                new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9)
        );

        double time = 0;

        // Add enough measurements to fill up the buffer
        for (; time < 4; time += 0.02) {
            estimator.updateWithTime(
                    time,
                    new Rotation2d(),
                    new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
                            new SwerveModulePosition()}
            );
        }

        // ConcurrentModificationException
        estimator.addVisionMeasurement(new Pose2d(new Translation2d(10, 10), new Rotation2d(0.1)), 1,
                VecBuilder.fill(0.1, 0.1, 0.1)
        );

        /*
        java.util.ConcurrentModificationException
            at java.base/java.util.TreeMap$NavigableSubMap$SubMapIterator.nextEntry(TreeMap.java:2016)
            at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(TreeMap.java:2064)
            at java.base/java.util.TreeMap$NavigableSubMap$SubMapEntryIterator.next(TreeMap.java:2058)
            at edu.wpi.first.math.estimator.SwerveDrivePoseEstimator.addVisionMeasurement(SwerveDrivePoseEstimator.java:213)
            at edu.wpi.first.math.estimator.SwerveDrivePoseEstimator.addVisionMeasurement(SwerveDrivePoseEstimator.java:250)
            at frc.robot.PoseEstimatorTest.testPoseEstimator(PoseEstimatorTest.java:55)
         */

    }
}
