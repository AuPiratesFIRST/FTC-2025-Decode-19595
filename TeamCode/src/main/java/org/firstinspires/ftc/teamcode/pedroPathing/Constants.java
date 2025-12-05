package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20)
            .forwardZeroPowerAcceleration(-98.92556584022245)
            .lateralZeroPowerAcceleration(-220.47273489994942)

            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.84364322833146)
            .yVelocity(98.32909370940584)
            ;
    public static DriveEncoderConstants localizerConstants =
            new DriveEncoderConstants()
                    .robotWidth(17)
                    .robotLength(17)
                    .leftFrontMotorName("leftFront")
                    .rightFrontMotorName("rightFront")
                    .leftRearMotorName("leftRear")
                    .rightRearMotorName("rightRear")
                    .leftFrontEncoderDirection(Encoder.REVERSE)
                    .leftRearEncoderDirection(Encoder.REVERSE)
                    .rightFrontEncoderDirection(Encoder.FORWARD)
                    .rightRearEncoderDirection(Encoder.FORWARD)
                    .forwardTicksToInches(0.006858033813605822)
                    .strafeTicksToInches(0.00974991185)
                    .turnTicksToInches(0.01381064545)
                    ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
