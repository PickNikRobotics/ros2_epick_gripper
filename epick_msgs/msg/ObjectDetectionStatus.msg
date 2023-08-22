# Reports whether an object has been gripped or not.
int8 status

int8 UNKNOWN=0                          # Unknown object detection. Regulating towards requested vacuum/pressure.
int8 OBJECT_DETECTED_AT_MIN_PRESSURE=1  # Object detected. Minimum vacuum value reached.
int8 OBJECT_DETECTED_AT_MAX_PRESSURE=2  # Object detected. Maximum vacuum value reached.
int8 NO_OBJECT_DETECTED=3               # Object loss, dropped or gripping timeout reached.
