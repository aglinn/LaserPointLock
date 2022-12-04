class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2


CHOSEN_TRIGGER = TriggerType.SOFTWARE
if isinstance(CHOSEN_TRIGGER, int):
    print(CHOSEN_TRIGGER)
