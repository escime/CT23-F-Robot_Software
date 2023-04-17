import math

import commands2
from wpilib import AddressableLED, Timer


class LEDs(commands2.SubsystemBase):

    animation_delay = 50
    timer = Timer()
    m_pattern = None
    record_time = None
    rainbow_pattern = []
    default_pattern = []
    for i in range(0, 29):
        rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
        rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
        rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
        rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
        rainbow_pattern.append(AddressableLED.LEDData(255, 0, 238))
    purple_pattern = [AddressableLED.LEDData(149, 50, 168)] * 10
    for i in range(0, 135):
        purple_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def __init__(self, port: int, length: int, animation_speed: float) -> None:
        super().__init__()
        self.m_led = AddressableLED(port)
        self.m_led.setLength(length)
        self.m_ledBuffer = [AddressableLED.LEDData(0, 0, 0)] * length
        self.default_pattern = [AddressableLED.LEDData(0, 0, 0)] * length
        self.m_led.setData(self.m_ledBuffer)
        self.m_led.start()
        self.animation_delay = animation_speed
        self.timer.start()
        self.record_time = self.timer.get()

    def rainbow_shift(self, state):
        # TODO Break out state check to its own function. Using the rainbow_shift as a catch-all is gimmicky af.
        if state:
            if self.timer.get() - self.animation_delay > self.record_time:
                self.m_ledBuffer = self.rainbow_pattern
                self.rainbow_pattern = self.rainbow_pattern[1:] + self.rainbow_pattern[:1]
                self.record_time = self.timer.get()
        else:
            self.m_ledBuffer = self.default_pattern
        self.m_led.setData(self.m_ledBuffer)

    def purple_chaser(self):
        if self.timer.get() - self.animation_delay > self.record_time:
            self.m_ledBuffer = self.purple_pattern
            self.purple_pattern = self.purple_pattern[1:] + self.purple_pattern[:1]
            self.record_time = self.timer.get()
        self.m_led.setData(self.m_ledBuffer)

    def heading_lock(self, heading):
        pos = int(-1 * (heading * 145 / 360) + 145/2)
        if pos > 143:
            pos = 143
        if pos < 3:
            pos = 3
        heading_pattern = [AddressableLED.LEDData(0, 0, 0)] * (pos - 2)
        heading_pattern = heading_pattern + [AddressableLED.LEDData(255, 0, 0)] * 5
        heading_pattern = heading_pattern + [AddressableLED.LEDData(0, 0, 0)] * (145-len(heading_pattern))
        self.m_led.setData(heading_pattern)

