# arduino_e_throttle
electronic throttle

(Optional) We proceed to more complex construction, such as an electronic throttle of BMW E46 1.8i 2002. We had to control the position and the speed of the throttle with a potensiometer (10kΩ). The potensiometer works like a gas pedal in real time, the faster we rotate the potensiometer, the faster opens the throttle holding the position steady. To hold the position steady we used the TPS sensor, knowing the current position of the throttle and a PID controller to stabilize the system and reduce the oscillations.
