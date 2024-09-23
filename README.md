An autonomous UV-C disinfecting car made using an Arduino UNO and other auxillary parts (motors, Sonar, IR sensors, etc.)

Uses efficient PID mechanism from FastPID library, applied at regular intervals using an ISR.
2 modes - Mapping and Normal Mode.
Data from Mapping mode is saved to EEPROM of Arduino UNO.
Saved data is then used to navigate in normal mode, but any humans etc. are avoided/circumvented surccessfully.

Overall very cost-effective design using no additional memory card or other complex/costly components.

Ensure that you have the required library files before compiling and writing to the Arduino UNO.
