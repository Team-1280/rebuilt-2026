package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * OdometryDrivetrain extends the base swerve drivetrain with logic that
 * dynamically determines how
 * much the robot should trust odometry vs vision.
 *
 * <p>
 * This class solves a specific mechanical problem: when swerve modules lift off
 * the ground
 * 
 * (e.g., over bumps), wheel encoders report false motion ("slip"). This
 * implementation detects slip
 * by 
 * comparing rotation rates from:
 *
 * <ul>
 * i>Gyroscopes (inertial, reliable during lift)
 * 
 * coder-based odometry (unreliable during lift)
 * 
 * 
 * 
 * 
 * namically adjusts trust in vision measurements accordingly.
 * 
 * 
 * 
 * 
 * 
 * This class is designed to be readable by rookies:
 *
 * <ul
 * >
 * 
 * <li>All math is explained at a high level
 * 
 * <li>No assumptions about probability or estimation theory
 * 
 * <li>Comments explain "why", not just "what"
 * </u
 * l>
 */
public final class OdometryDrivetrain extends CommandSwerveDrivetrain {
 
      /**
       * How often odometry is updated, in Hz.
         *
         * <p>
         * This should match the CAN update rate of the drivetrain hard

            private static final double ODOMETRY_UPDATE_FREQUENCY = CommandSwer

           
         * /**
         * 
         
             * Vision standard deviations when vision is performing very well.

             * <p>

           
             * <p>
         * 
             * Order is: [x meters, y meters, rota
           
            private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.

           
         * /**
         * 
         
             * Vision standard deviations when vision is performing very poorly.

             * <p>
             * Higher values mean "trust vision less".

            private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(0
        

             * Acceptable disagreement between two gyro angular velocity measur

           
             *
         * 
          
             *
         *  Units are radians per second.
         * 
             */
           
        

             * Acceptable disagreement between odometry-derived rotation rate a

           
             * <p>
          
             *
         *  threshold
         * 
             * detects that slip condition.
         * 
             *
          
             *
         *  Units are radians per second.
           

             private static l double SLIP_DETECTION_THRESHOLD = 2.0;
        // 

           
            private static final double TRUST_VISION_RANGE_MIN = 0.25;
         
            /*
         * * Maximum distance at which vision measurements are trusted, in met
         * rs. */
            private static final double TRUST_VISION_RANGE_MAX = 3.5;
         
            /**

           
             * <p>
          
             *
         * /
         * 
            private final DoubleSupplier pigeonRateSupplier;
         * 
         
           
            private final DoubleSupplier rioRateSupplier;

            /**

             */

        
         * 
         * 
         
            /** FPGA timestamp from the previous loop iteration. *

        
            /**

             *

           
         *  * 1.0 means "trust odometry fully" (no slip detected), 0.0 means 
         * do not trust
         
             * odometry"

           
         *  */
         
            private double cachedOdometryTrust = 

            /**
             * Whether slip was detected in the most re

           
             * Used for telemetry and potential future recovery logic.
          
            pr
         * ivate boolean slipDetected = false;
         * 
         
         * 
           
             * Constructs the drivetra
             *
                 * @pa
                                m rioRateSupplier    angular veloci
                                
                                dometryDrivetrain(DoubleS
                                super(
                                                TunerCon
                                                ODOMETRY_U

                                        TunerConstants.FrontRight,
                                        TunerConstants.BackLeft,
                                        TunerConstants.BackRight);
                        this.pigeonRateSupplier = pigeonRateSupplier;
                        this.rioRateSupplier = rioRateSu
                            lastPose = getState().Pose; // Initialize with actual starting pose
                }

                /**
                 * Computes a trust value using exponential decay.
         

           
             *
          
             *
         *  @param sigma how tolerant we are of error (larger = more tolerant)
          
             */
            private static double gaussianTrust(double error, double sigma) {
                    return Math.exp(-Math.abs(
           
        
                /**
         

           
             * alpha = 0 returns worst, alpha = 1 returns best.
          
            pr
         * ivate static Matrix<N3, N1> interpolateMatrices(
           
                    return VecBuilder.fill(
                                            MathUtil.interpolate(worst.get(0, 0), 
                                       
                                                MathUtil.interpolate(worst.get(2, 0), best.ge
                                
                                
         

                 
        
                        // Comput

                        double dt = now - lastTimeSec;
                        lastTimeSec = now;
                                
                                // Protect against divi
                                if (dt <= 1e-4 || Doubl
                                        return;
                                }

                        Pose2d currentPose = getState().Pose;
                
                        // Read angular velocities from both gyro sources
                        double omegaPigeon = pigeonRateSupplier.getAsDouble(); // Pig
                        double omegaRio = rioRateSupplier.getAsDoub
                

                        double gyroAgreement = Math.abs(omegaPigeon - omegaRio);
                        double gyroAgreementTrust = gaussianTrust(gyroAgreement, GYRO_
                
                        // Blend gyro readings based on agreement - 
                        // rotation rate

                
                        // Compute rotation rate derived purely from encoder odomet
                        // slip)
                        double deltaTheta = MathUtil.angleModulus(

                                                        - lastPose.getRotation().getRa
                        double omegaOdometry
                
                        // DETECT SLIP: When wheels lift, encoder odometry reports fals
                        // while gyros (inertial sensors) remain accurate. Large dis
                        double slipError = Math.abs(omegaOdometry - omegaInertial);
                        slipDetected = slipError > 
                
                        // Trust odometry less when slip is detected
                      double odometryTrust = gaussianTrust(slipError, SLIP_DE
                      cachedOdometryTrust = odometryTrust;
                
                        // Update telemetry 
                        Logger.recordOutput("Odometry/Po
                            Logger.recordOutput("Odometry/Trust", odometryTrust);
                            Logger.recordOutput("Odometry/SlipDetected", slipDe
                            Logger.recordOutput("Odometry/Omega/Inertial", omegaIner
                 

                        Logger.recordOutput("Odometry/Omega/Rio", omegaRio);
                        Logger.recordOutput("Odometry/Omega/D
                
                        // Prepare for next iteration
                        lastPose = currentPose;
                }
                
                /**

                 * odometry
                 * reliability.
                 *
                 * <p>
                 * sion is rejected if:
                 * 
                 * l>
                 *
                 * <li>The target is too close (parallax errors dominate)
                 * <li>The target is too far (low resolution, high noise)
                 * <li>Odometry is unreliable (slip detected) AND 

                 * accurately correct for latency without trustworthy odometry)
                 * </ul>
                 *
                 * <p>
                 * When odometry is unreliable due to slip, we increase vision sta
                 * deviations (reduce
                 * trust) because:
                 *
                 * <ul>
                 * <li>Vision timestamp correction relies on odometry to estimate
                 * during latency
         

           
             *
         * 
             * @param p
          
             *
         *                       
          
             * 
           */
           ublic void addVisionMeasurement(
                          Pose2d pose, double timestampSeconds, d
                  // REJECT: Obviously bad measurements
         * 
              if (noisy
                
          
              
         *               Logger.recordOutput("Vision/Rejected/Reason", "RangeOr
         * oisy");
                          
          
         
                  // REJECT: Vision measurements that arrived too late when odometry i
         *  unreliable
                  // Reason: We can't accurately correct for latency without trustw
         * rthy odome
                
          
                                        Logger.recordOutput("Vision
                            return;
                    }   
                
           
                    // because timestamp 
                            // trustworthy,
                        // we can fully trust well-co
                        d
                                Matrix<N3, N1> visionStd = interpolateMatr
                                                VISION_STD_WORST, VISION_STD_
                        
                            // 
                 

                        Logger.recordOutput("Vision/Accepted/StdDev
                        Logger.recordOutput("Vision/Accepted/StdDev/Y", visionStd.get(1, 0
                // );
                        Logger.recordOutput("Vision/Accepted/StdDev/Theta", vision
                        Logger.recordOutput("Vision/Accepted/TrustFactor", vi
                }
                        
                        
                 

                 * <p>
                // 
                 * 1.0 = fully trustworthy (no slip), 0.0 = completely untrustworthy 
                 * slip).
                 *
                 * <p>
                 * Useful for other subsys ble.
                                

                 */
                public double getOdometryTrust() {
                        return cachedOdometryTrust;
                }
                
                /**
                 * Returns whether wheel slip was detected in the most recent update c
         

           
             * ground,
          
             *
         * 
         * 
             * @return true if sl
          
            public boolean isSlipDetected() {
         *         
           
            }
                
        

        
                    
         * 
         
            
         *         
         
             
                
         

           
         
         * 
          
         
         * 
         * 
         
