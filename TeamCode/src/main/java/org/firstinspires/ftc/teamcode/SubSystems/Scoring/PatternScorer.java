package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

/**
 * Pattern scorer for the DECODE game
 * Scores patterns based on motif matching against detected ramp artifacts
 */
public class PatternScorer {

    private final ArtifactColor[] motif;
    private final ArtifactColor[] pattern;

    /**
     * Create a pattern scorer with a 3-color motif
     * 
     * @param motif 3-color motif array
     * @throws IllegalArgumentException if motif is not exactly 3 colors
     */
    public PatternScorer(ArtifactColor[] motif) {
        if (motif == null || motif.length != 3) {
            throw new IllegalArgumentException("Motif must have exactly 3 colors");
        }
        this.motif = motif.clone();
        this.pattern = generatePattern(motif);
    }

    /**
     * Generate the full 9-position pattern from the 3-color motif
     * 
     * @param motif 3-color motif
     * @return 9-position pattern
     */
    private ArtifactColor[] generatePattern(ArtifactColor[] motif) {
        ArtifactColor[] pattern = new ArtifactColor[9];
        for (int i = 0; i < 9; i++) {
            pattern[i] = motif[i % 3];
        }
        return pattern;
    }

    /**
     * Score a detected ramp pattern against the expected pattern
     * 
     * @param detectedRamp Array of detected artifact colors
     * @return Number of matches (0-9)
     */
    public int scorePattern(ArtifactColor[] detectedRamp) {
        if (detectedRamp == null || detectedRamp.length == 0) {
            return 0;
        }

        int matches = 0;
        int maxLength = Math.min(detectedRamp.length, pattern.length);

        for (int i = 0; i < maxLength; i++) {
            if (detectedRamp[i] == pattern[i]) {
                matches++;
            }
        }

        return matches;
    }

    /**
     * Get the full 9-position pattern
     * 
     * @return Pattern array
     */
    public ArtifactColor[] getPattern() {
        return pattern.clone();
    }

    /**
     * Get the original motif
     * 
     * @return Motif array
     */
    public ArtifactColor[] getMotif() {
        return motif.clone();
    }

    /**
     * Convert a pattern array to a string representation
     * 
     * @param pattern Pattern array
     * @return String representation (e.g., "GPPGPPGPP")
     */
    public static String patternString(ArtifactColor[] pattern) {
        if (pattern == null) {
            return "";
        }

        StringBuilder sb = new StringBuilder();
        for (ArtifactColor color : pattern) {
            sb.append(color.getCharacter());
        }
        return sb.toString();
    }

    /**
     * Get string representation of this pattern scorer
     * 
     * @return String representation
     */
    @Override
    public String toString() {
        return "PatternScorer{motif=" + patternString(motif) + ", pattern=" + patternString(pattern) + "}";
    }
}
