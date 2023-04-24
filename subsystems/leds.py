import commands2
from wpilib import AddressableLED, Timer


class LEDs(commands2.SubsystemBase):

    animation_delay = 50
    timer = Timer()
    m_pattern = None
    record_time = None
    rainbow_pattern = []
    default_pattern = []
    purple_pattern = []
    notifier_length = 3
    notifier_state = [AddressableLED.LEDData(255, 0, 0)] * notifier_length

    def __init__(self, port: int, length: int, num: int, animation_speed: float) -> None:
        super().__init__()
        self.m_led = AddressableLED(port)  # Connect the LED chain to the right port.
        self.length = length  # Set the length from the constructor.
        self.num_of_strips = num  # Set the number of strips from the constructor.
        self.m_led.setLength(length * num)  # Set the WPILIB length for the entire LED chain.
        self.m_ledBuffer = [AddressableLED.LEDData(0, 0, 0)] * length  # Set up the buffer.
        self.clear_pattern = [AddressableLED.LEDData(0, 0, 0)] * length  # Set up the blank pattern.
        repeat = self.num_of_strips - 1
        if repeat == 0:
            self.m_ledBuffer_complete = self.m_ledBuffer
        if repeat > 0:
            self.m_ledBuffer_complete = self.m_ledBuffer * self.num_of_strips
        self.m_led.setData(self.m_ledBuffer_complete)
        self.m_led.start()
        self.animation_delay = animation_speed
        self.timer.start()
        self.record_time = self.timer.get()

        # Setup rainbow pattern default
        for i in range(0, int(self.length / 5)):
            self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(255, 213, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(0, 255, 0))
            self.rainbow_pattern.append(AddressableLED.LEDData(0, 47, 255))
            self.rainbow_pattern.append(AddressableLED.LEDData(255, 0, 238))

        # Setup purple chase pattern default
        self.purple_pattern = [AddressableLED.LEDData(149, 50, 168)] * 10
        for i in range(0, self.length - 10):
            self.purple_pattern.append(AddressableLED.LEDData(0, 0, 0))

    def clear_buffer(self) -> None:
        """Clear the master buffer of all data."""
        self.m_ledBuffer = self.clear_pattern

    def set_chain(self) -> None:
        """Configure the set of LED chains with the pattern stored in the buffer."""
        repeat = self.num_of_strips - 1
        if repeat == 0:
            self.m_ledBuffer_complete = self.m_ledBuffer
        if repeat > 0:
            self.m_ledBuffer_complete = self.m_ledBuffer * self.num_of_strips
        self.m_led.setData(self.m_ledBuffer_complete)

    def set_chain_with_notifier(self):
        """Configure the chain with a modified buffer that includes a properly positioned notifier state."""
        self.m_ledBuffer[-self.notifier_length:] = self.notifier_state
        self.set_chain()

    def set_notifier(self, state: str):
        """Set the notifier state. This is intended to be a second layer of information on top of the master LED
        code that can give a secondary status update."""
        if state == "GREEN":
            self.notifier_state = [AddressableLED.LEDData(0, 255, 0)] * self.notifier_length
        if state == "BLUE":
            self.notifier_state = [AddressableLED.LEDData(0, 0, 255)] * self.notifier_length
        if state == "RED":
            self.notifier_state = [AddressableLED.LEDData(255, 0, 0)] * self.notifier_length

    def rainbow_shift(self):
        """Configure the LED code for a rainbow wrapping around each strip."""
        if self.timer.get() - self.animation_delay > self.record_time:
            self.m_ledBuffer = self.rainbow_pattern
            self.rainbow_pattern = self.rainbow_pattern[1:] + self.rainbow_pattern[:1]
            self.record_time = self.timer.get()
        self.set_chain_with_notifier()

    def purple_chaser(self):
        """Configure the LED code for a purple chaser (2023 Charged Up Default)."""
        if self.timer.get() - self.animation_delay > self.record_time:
            self.m_ledBuffer = self.purple_pattern
            self.purple_pattern = self.purple_pattern[1:] + self.purple_pattern[:1]
            self.record_time = self.timer.get()
        self.set_chain_with_notifier()

    def heading_lock(self, heading):
        """Configure the LED code for a demo of robot heading tracking."""
        pos = int(-1 * (heading * self.length / 360) + self.length/2)
        if pos > 143:
            pos = 143
        if pos < 3:
            pos = 3
        heading_pattern = [AddressableLED.LEDData(0, 0, 0)] * (pos - 2)
        heading_pattern = heading_pattern + [AddressableLED.LEDData(255, 0, 0)] * 5
        heading_pattern = heading_pattern + [AddressableLED.LEDData(0, 0, 0)] * (self.length-len(heading_pattern))
        self.m_ledBuffer = heading_pattern
        self.set_chain()
