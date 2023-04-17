import wpilib


class CustomHID:
    # Create a controller object default (arbitrarily selected Xbox on port 0)
    controller: wpilib.XboxController
    controller_type: str

    def __init__(self, port: int, hid: str) -> None:
        super().__init__()
        if hid == "xbox":
            self.controller = wpilib.XboxController(port)
            self.controller_type = "xbox"
        if hid == "ps4":
            self.controller = wpilib.PS4Controller(port)
            self.controller_type = "ps4"
        else:
            self.controller = wpilib.Joystick(port)
            self.controller_type = "generic"

    def get_button(self, button: str) -> bool:
        value = False
        if self.controller_type == "xbox":
            if button == "A":
                value = self.controller.button(0)
            if button == "B":
                value = self.controller.button(1)
            if button == "X":
                value = self.controller.button(2)
            if button == "Y":
                value = self.controller.button(3)
            if button == "LB":
                value = self.controller.button(4)
            if button == "RB":
                value = self.controller.button(5)
            if button == "VIEW":
                value = self.controller.button(6)
            if button == "MENU":
                value = self.controller.button(7)
            if button == "LTHUMB":
                value = self.controller.button(8)
            if button == "RTHUMB":
                value = self.controller.button(9)
        return value

    def get_trigger(self, trigger: str, threshold: float) -> bool:
        value = False
        if self.controller_type == "xbox":
            if trigger == "R":
                print("checkin")
                if self.controller.getRawAxis(3) >= threshold:
                    value = True
            if trigger == "L":
                if self.controller.getRawAxis(2) >= threshold:
                    value = True
        return value

    def get_axis(self, axis: str, deadband: float) -> float:
        value = 0.0
        if self.controller_type == "xbox":
            if axis == "LX":
                if self.controller.getRawAxis(0) >= deadband:
                    value = self.controller.getRawAxis(0)
            if axis == "LY":
                if self.controller.getRawAxis(1) >= deadband:
                    value = self.controller.getRawAxis(1)
            if axis == "RX":
                if self.controller.getRawAxis(4) >= deadband:
                    value = self.controller.getRawAxis(4)
            if axis == "RY":
                if self.controller.getRawAxis(5) >= deadband:
                    value = self.controller.getRawAxis(5)
        return value

    def get_d_pad(self) -> str:
        value = "Z"
        if self.controller.getPOV() == 0:
            value = "N"
        if self.controller.getPOV() == 45:
            value = "NE"
        if self.controller.getPOV() == 90:
            value = "E"
        if self.controller.getPOV() == 135:
            value = "SE"
        if self.controller.getPOV() == 180:
            value = "S"
        if self.controller.getPOV() == 225:
            value = "SW"
        if self.controller.getPOV() == 270:
            value = "W"
        if self.controller.getPOV() == 315:
            value = "NW"
        return value

    def set_rumble(self, strength: float) -> None:
        self.controller.setRumble(wpilib.XboxController.RumbleType.kBothRumble, strength)

    def get_xbox_controller(self) -> wpilib.XboxController:
        return self.controller
