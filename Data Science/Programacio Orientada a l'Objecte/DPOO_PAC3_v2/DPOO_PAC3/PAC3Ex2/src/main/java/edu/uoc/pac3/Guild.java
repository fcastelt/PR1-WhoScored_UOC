package edu.uoc.pac3;
import java.time.LocalDate;
public class Guild {
    private String name;
    private int level;
    private String description;
    private LocalDate creationDate;
    private boolean recruiting;
    private int numMembers = 0;
    private int sumLevels = 0;
    private int NUM_MAX_MEMBERS;
    private int nextId = 1;

    public static final String INVALID_NAME = "[ERROR] The name cannot be null, has less than the minimum umber of characters, exceeds the maximum number of characters or contains only whitespaces";
    public static final String INVALID_DESCRIPTION= "[ERROR] The description cannot be null and cannot exceed the predefined maximum number of characters.";
    public static final String INVALID_CREATION_DATE = "[ERROR] The creation date cannot be null or in the future.";
    public static final String INVALID_MAX_MEMBERS = "[ERROR] The number of members cannot exceed the predefined maximum.";
    public static final String MEMBER_NULL = "[ERROR] The member cannot be null";
    public static final String MEMBER_ALREADY_EXISTS = "[ERROR] The member already exists in the guild";
    public static final String MEMBER_NOT_FOUND = "[ERROR] The member does not exist in the guild";
    public static final String MEMBER_NO_PET = "[ERROR] The member does not have a pet";
    private static final int MIN_NAME_LENGTH = 5;
    private static final int MAX_NAME_LENGTH = 25;
    private static final int MAX_LEVEL = 20;
    private static final int MAX_DESCRIPTION_LENGTH = 100;

    public Guild(String name, int level, String description, LocalDate creationDate, boolean recruiting, int num_max_members) {
        setName(name);
        setLevel(level);
        setDescription(description);
        setCreationDate(creationDate);
        setRecruiting(recruiting);
        setNUM_MAX_MEMBERS(num_max_members);
    }
    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int level) {
        this.level = level;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

    public LocalDate getCreationDate() {
        return creationDate;
    }

    public void setCreationDate(LocalDate creationDate) {
        this.creationDate = creationDate;
    }

    public boolean isRecruiting() {
        return recruiting;
    }

    public void setRecruiting(boolean recruiting) {
        this.recruiting = recruiting;
    }

    public int getNumMembers() {
        return numMembers;
    }

    public void incNextId(){
        nextId++;
    }



    public void setNUM_MAX_MEMBERS(int NUM_MAX_MEMBERS) {
        this.NUM_MAX_MEMBERS = NUM_MAX_MEMBERS;
    }





}
