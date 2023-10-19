from wpilib import CAN  # CANData, CANStatus


class AMHBE:
    """TEST"""
    can_id = None

    def __init__(self, can_id):
        super().__init__()
        self.can_id = can_id

    def set_absolute(self):
        CAN.writePacket(self.can_id, 1, 1234)

    def set_relative(self):
        CAN.writePacket(self.can_id, 0, 1234)
