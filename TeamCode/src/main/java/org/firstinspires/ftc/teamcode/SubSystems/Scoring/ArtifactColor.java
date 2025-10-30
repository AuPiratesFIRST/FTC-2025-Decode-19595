package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

/**
 * Enumeration for artifact colors in the DECODE game
 * Represents the two possible colors of artifacts: GREEN and PURPLE
 */
public enum ArtifactColor {
    GREEN('G'),
    PURPLE('P');

    private final char character;

    ArtifactColor(char character) {
        this.character = character;
    }

    /**
     * Get the character representation of this color
     * 
     * @return Character representation ('G' or 'P')
     */
    public char getCharacter() {
        return character;
    }

    /**
     * Create an ArtifactColor from a character
     * 
     * @param character Character representation ('G' or 'P', case insensitive)
     * @return ArtifactColor enum value
     * @throws IllegalArgumentException if character is not 'G' or 'P'
     */
    public static ArtifactColor fromChar(char character) {
        char upperChar = Character.toUpperCase(character);
        switch (upperChar) {
            case 'G':
                return GREEN;
            case 'P':
                return PURPLE;
            default:
                throw new IllegalArgumentException("Invalid artifact color: " + character + ". Must be 'G' or 'P'");
        }
    }

    @Override
    public String toString() {
        return name();
    }
}
