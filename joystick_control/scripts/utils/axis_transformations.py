def deadzone(axis, deadzone):
    return (
        axis * (1 - deadzone / abs(axis)) / (1 - deadzone)
        if abs(axis) > deadzone
        else 0.0
    )
