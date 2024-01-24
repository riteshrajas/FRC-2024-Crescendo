Intake
    tasks
        set position
            stow,
            ground intake,
            amp,
            trap,
        intake note from ground
        pass note to indexer
        eject note
            amp,
            trap,
    data
        has note
        current position

indexer
    tasks
        Stow note
        pass note to shooter
        pass note to intake (not sure if needed)
    data
        is stowed
        has note

shooter
    tasks
        shoot note
        spin up flywheel
        stop flywheel
    data
        at speed

arm
    task
        set position
            stow,
            amp,
            trap,
        set position for speaker
            subwoofer,
            podium,
            wing (may not be needeed),
    data
        get position


