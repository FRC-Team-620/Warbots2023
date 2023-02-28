// package org.jmhsrobotics.frc2023.util;

// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.SPI;

// public class SusanDrivePorts implements IDrivePorts {

// public final int leftFrontMotorCANId = 4;
// public final int rightFrontMotorCANId = 2;
// public final int leftRearMotorCANId = 3;
// public final int rightRearMotorCANId = 1;

// public final boolean rightFrontMotorInversion = false;
// public final boolean leftFrontMotorInversion = true;

// public final double wheelDiameterInInches = 6;
// public final IIMUWrapper imu = new NavxIMU(SPI.Port.kMXP);

// public final double maxVelocity = 2;
// public final double maxAcceleration = 2;

// public ProfiledPIDConfig autoDistanceProfiledPID = new ProfiledPIDConfig(2,
// 0.2, 0.0,
// new Constraints(maxVelocity, maxAcceleration));
// public PIDConfig keepHeadingPID = new PIDConfig(0.010, 0.010, 0);
// public PIDConfig balancingPID = new PIDConfig(0.4, 0.2, 0);

// public final double balanceCreepSpeed = 0.1;

// @Override
// public ProfiledPIDConfig getAutoDistanceProfiledPID() {
// // TODO Auto-generated method stub
// return autoDistanceProfiledPID;
// }
// @Override
// public PIDConfig getKeepHeadingPID() {
// // TODO Auto-generated method stub
// return keepHeadingPID;
// }
// @Override
// public PIDConfig getBalancingPID() {
// // TODO Auto-generated method stub
// return balancingPID;
// }

// @Override
// public int getLeftFrontMotorCANId() {
// return leftFrontMotorCANId;
// }
// @Override
// public int getRightFrontMotorCANId() {
// return rightFrontMotorCANId;
// }
// @Override
// public int getLeftRearMotorCANId() {
// return leftRearMotorCANId;
// }
// @Override
// public int getRightRearMotorCANId() {
// return rightRearMotorCANId;
// }
// @Override
// public boolean getRightFrontMotorInversion() {
// return rightFrontMotorInversion;
// }
// @Override
// public boolean getLeftFrontMotorInversion() {
// return leftFrontMotorInversion;
// }
// @Override
// public double getWheelDiameterInInches() {
// return wheelDiameterInInches;
// }

// @Override
// public double getMaxVelocity() {
// // TODO Auto-generated method stub
// return maxVelocity;
// }
// @Override
// public double getMaxAcceleration() {
// // TODO Auto-generated method stub
// return maxAcceleration;
// }
// @Override
// public double getBalanceCreepSpeed() {
// // TODO Auto-generated method stub
// return balanceCreepSpeed;
// }

// @Override
// public IIMUWrapper getIMU() {
// return imu;
// }
// }
