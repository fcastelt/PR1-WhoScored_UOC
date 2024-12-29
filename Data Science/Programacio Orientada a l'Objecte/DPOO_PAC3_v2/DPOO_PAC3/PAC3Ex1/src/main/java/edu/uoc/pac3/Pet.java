package edu.uoc.pac3;

import java.time.LocalDate;

public class Pet {
    private String name;
    private int level;
    private LocalDate birthdate;
    private int loyalty;
    private int stamina;
    private boolean aggressive;

    // Final attributes
    private static final int MIN_NAME_LENGTH = 3;
    private static final int MAX_NAME_LENGTH = 20;
    private static final int MAX_LEVEL = 60;
    public static final String INVALID_NAME = "[ERROR] The name must not be null, empty, consist solely of spaces, and it must be within the predefined minimum and maximum character limits.";
    public static final String INVALID_LEVEL = "[ERROR] The level must be between 1 and the predefined maximum.";
    public static final String INVALID_BIRTHDATE = "[ERROR] The birthday date cannot be null or in the future.";
    public static final String INVALID_LOYALTY = "[ERROR] Loyalty must be within 0 and 100.";
    public static final String INVALID_STAMINA = "[ERROR] Stamina must be within 0 and 100.";

    // Constructor
    public Pet(String name, int level, LocalDate birthdate, int loyalty, int stamina, boolean aggressive) {
        setName(name);
        setLevel(level);
        setBirthdate(birthdate);
        setLoyalty(loyalty);
        setStamina(stamina);
        setAggressive(aggressive);
    }

    // Setters and getters
    public String getName() {
        return name;
    }

    private void setName(String name) throws IllegalArgumentException {
        if (name == null || name.trim().isEmpty() || name.trim().length() < MIN_NAME_LENGTH || name.trim().length() > MAX_NAME_LENGTH) {
            throw new IllegalArgumentException(INVALID_NAME);
        }
        this.name = name.trim();
    }



    public int getLevel() {
        return level;
    }

    private void setLevel(int level) throws IllegalArgumentException{
        if (level < 1 || level > MAX_LEVEL) {
            throw new IllegalArgumentException(INVALID_LEVEL);
        }
        this.level = level;
    }


    public LocalDate getBirthdate() {
        return this.birthdate;
    }

    private void setBirthdate(LocalDate birthdate) throws IllegalArgumentException{
        if (birthdate != null && !birthdate.isAfter(LocalDate.now())) {
            this.birthdate = birthdate;
        } else {
            throw new IllegalArgumentException(INVALID_BIRTHDATE);
        }
    }
    public int getLoyalty() {
        return this.loyalty;
    }

    private void setLoyalty(int loyalty) throws IllegalArgumentException{
        if (loyalty < 0 || loyalty > 100) {
            throw new IllegalArgumentException(INVALID_LOYALTY);
        }
        this.loyalty = loyalty;
    }


    public int getStamina() {
        return stamina;
    }

    private void setStamina(int stamina) throws IllegalArgumentException{
        if (stamina < 0 || stamina > 100) {
            throw new IllegalArgumentException(INVALID_STAMINA);
        }
        this.stamina = stamina;
    }

    public boolean isAggressive() {
        return aggressive;
    }

    private void setAggressive(boolean aggressive) {
        this.aggressive = aggressive;
    }
}

