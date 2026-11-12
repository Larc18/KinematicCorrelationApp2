package de.cwelsch.kinematiccorrelationapp.services.timenormalizationservice.strategy;

import de.cwelsch.kinematiccorrelationapp.model.FrameData;
import de.cwelsch.kinematiccorrelationapp.model.Joints;

import java.util.List;

/**
 * Refactored Movement Strategy for Becken-Rumpf-Rotation (BRR).
 * Employs peak-centered relative ROM thresholds on rotation angles (Lumbar/Pelvis Rotation)
 * to capture the full oscillation cycle (neutral -> left rotation -> right rotation -> neutral)
 * with robust, rate-independent cross-system synchronization.
 */
public class BBRStrategy implements MovementStrategy {

    private static final double ROM_ONSET_RATIO = 0.05; // 5% Schwellenwert der Rotations-Amplitude
    private static final double SECOND_PEAK_SEARCH_BUFFER_SECONDS = 20.0 / 85.0;

    /**
     * Detects the pelvis/trunk rotation movement window by isolating the first significant rotation
     * excursion, locating the opposing second peak, and expanding to where the rotation angle returns
     * within a 5%-of-ROM threshold of baseline. Falls back to the entire trial if no rotation joint
     * data, sufficient amplitude, or plausible window boundary is found.
     *
     * @param frames the sequential tracking data points collected during the trial recording
     * @return the detected movement window, or a fallback window covering the entire trial
     */
    @Override
    public MovementWindow findMovementWindow(List<FrameData> frames) {
        if (frames == null || frames.size() < 10) return fallback(frames);

        boolean hasPelvisRotation = frames.stream()
                .anyMatch(f -> f.jointAngle().get(Joints.PELVIS_ROTATION) != null);
        boolean hasLumbarRotation = frames.stream()
                .anyMatch(f -> f.jointAngle().get(Joints.LUMBAR_ROTATION) != null);

        if (!hasPelvisRotation && !hasLumbarRotation) return fallback(frames);

        Joints rotationJoint = hasLumbarRotation ? Joints.LUMBAR_ROTATION : Joints.PELVIS_ROTATION;

        int baselineRange = Math.max(3, (int) (frames.size() * 0.05));
        double baselineRotation = 0.0;
        int validCount = 0;

        for (int i = 0; i < baselineRange; i++) {
            Double val = frames.get(i).jointAngle().get(rotationJoint);
            if (val != null && !Double.isNaN(val)) {
                baselineRotation += val;
                validCount++;
            }
        }
        baselineRotation = validCount > 0 ? baselineRotation / validCount : 0.0;

        double globalMaxAbsDeviation = 0.0;
        for (FrameData frame : frames) {
            Double rot = frame.jointAngle().get(rotationJoint);
            if (rot != null && !Double.isNaN(rot)) {
                globalMaxAbsDeviation = Math.max(globalMaxAbsDeviation, Math.abs(rot - baselineRotation));
            }
        }
        if (globalMaxAbsDeviation < 5.0) {
            return fallback(frames);
        }
        double thresholdDelta = globalMaxAbsDeviation * ROM_ONSET_RATIO;

        int firstPeakFrame = -1;
        double firstPeakDeviation = 0.0;
        boolean inExcursion = false;

        for (int i = 0; i < frames.size(); i++) {
            Double rot = frames.get(i).jointAngle().get(rotationJoint);
            if (rot == null || Double.isNaN(rot)) continue;
            double deviation = Math.abs(rot - baselineRotation);

            if (!inExcursion) {
                if (deviation > thresholdDelta) {
                    inExcursion = true;
                    firstPeakDeviation = deviation;
                    firstPeakFrame = i;
                }
            } else {
                if (deviation > firstPeakDeviation) {
                    firstPeakDeviation = deviation;
                    firstPeakFrame = i;
                }
                if (deviation <= thresholdDelta) {
                    break;
                }
            }
        }

        if (firstPeakFrame == -1) {
            return fallback(frames);
        }

        int startFrame = 0;
        for (int i = firstPeakFrame; i >= 0; i--) {
            Double rot = frames.get(i).jointAngle().get(rotationJoint);
            if (rot != null && !Double.isNaN(rot) && Math.abs(rot - baselineRotation) <= thresholdDelta) {
                startFrame = i;
                break;
            }
        }

        int secondPeakFrame = firstPeakFrame;
        double secondPeakDeviation = Double.NEGATIVE_INFINITY;

        int searchStart = frameOffsetForBuffer(frames, firstPeakFrame);
        for (int i = searchStart; i < frames.size(); i++) {
            Double rot = frames.get(i).jointAngle().get(rotationJoint);
            if (rot == null || Double.isNaN(rot)) continue;
            double deviation = Math.abs(rot - baselineRotation);
            if (deviation > secondPeakDeviation) {
                secondPeakDeviation = deviation;
                secondPeakFrame = i;
            }
        }

        int endFrame = frames.size() - 1;
        for (int i = secondPeakFrame; i < frames.size(); i++) {
            Double rot = frames.get(i).jointAngle().get(rotationJoint);
            if (rot != null && !Double.isNaN(rot) && Math.abs(rot - baselineRotation) <= thresholdDelta) {
                endFrame = i;
                break;
            }
        }

        if (startFrame >= endFrame) return fallback(frames);
        return new MovementWindow(
                frames.get(startFrame).second(),
                frames.get(endFrame).second(),
                false
        );
    }

    private MovementWindow fallback(List<FrameData> frames) {
        if (frames == null || frames.isEmpty()) return new MovementWindow(0.0, 0.0, true);
        return new MovementWindow(frames.get(0).second(), frames.get(frames.size() - 1).second(), true);
    }

    /**
     * Wandelt SECOND_PEAK_SEARCH_BUFFER_SECONDS anhand der tatsächlichen Framerate DIESES Trials
     * in eine Frame-Anzahl um, statt eine feste Frame-Anzahl (die je nach System unterschiedlich
     * viel Zeit bedeutet) zu verwenden.
     */
    private int frameOffsetForBuffer(List<FrameData> frames, int fromIndex) {
        double dt = (frames.size() > 1) ? (frames.get(1).second() - frames.get(0).second()) : (1.0 / 85.0);
        if (dt <= 0) dt = 1.0 / 85.0;
        int bufferFrames = Math.max(1, (int) Math.round(SECOND_PEAK_SEARCH_BUFFER_SECONDS / dt));
        return Math.min(fromIndex + bufferFrames, frames.size() - 1);
    }
}