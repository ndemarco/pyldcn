The Logosol CNC has a built in power supply that supplies control power constantly and switches motor power (UM) on and off for safety. The spindle power is possibly slightly differently controlled.

Many conditions must be satisfied before the motor power will enable. When those conditions are met, the system is 'ready to power on'. This can be determined from the diagnostic states table of the sk2310g2.

When ready to power on, power can be fully turned on by pressing and releasing the power button (CN15.5, CN15.6). If sk2310g2 jumper J21 is shorted closed, output 15 (byte1/bit7) can be pulsed on/off in place of pressing the physical power on button. 100ms pulse works.

Power on can be determined via properties in pyldcn, namely in io.py and subordinate functions. There is no bit showing power state.

Turning power off can only be done by software, unless pressing the emergency stop button is considered. Depending on the state of J2, the power off function will be delayed. During this delay, the diagnostic state will reflect 'powering off - waiting for the delay'. This delay can be <= 4s, so after 5s, there's an error thrown.

The power on and off logic is captured well in the pyldcn script test_power_control.py.

To turn the power on, once ready to power is set, pulse out15 for 100ms, then watch the diagnostic state. If the power is not on, you must prompt the user to turn on the power (the power switch LED will be flashing when the system is ready to be powered also). Prompt the user, then monitor the state for power on. Once power is on, proceed with the application.

When the user requests turning the power off, turn off the power via bit 15 and monitor the state. If the power isn't off in 5s, throw an error.