import signal

from ground_station.gamepad_input import GamepadReader, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED


class FakeJoystick:
    def __init__(self, axis_values):
        self._axis_values = axis_values
        self.initialized = False

    def init(self):
        self.initialized = True

    def get_axis(self, index):
        return self._axis_values[index]


class FakeJoystickModule:
    def __init__(self):
        self._count = 0
        self._joysticks = {}

    def init(self):
        pass

    def get_count(self):
        return self._count

    def Joystick(self, index):
        return self._joysticks[index]

    def plug_in(self, index, joystick):
        self._joysticks[index] = joystick
        self._count = max(self._count, index + 1)

    def unplug_all(self):
        self._count = 0
        self._joysticks = {}


class FakeEventModule:
    def pump(self):
        pass


class FakePygame:
    def __init__(self):
        self.joystick = FakeJoystickModule()
        self.event = FakeEventModule()

    def init(self):
        pass


def test_poll_returns_false_when_no_joystick_connected():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)

    assert reader.poll() is False


def test_poll_returns_true_once_joystick_is_plugged_in():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)

    fake_pygame.joystick.plug_in(0, FakeJoystick({LEFT_STICK_X: 0.0 for LEFT_STICK_X in range(4)}))

    assert reader.poll() is True


def test_poll_returns_false_after_joystick_unplugged():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)
    fake_pygame.joystick.plug_in(0, FakeJoystick({i: 0.0 for i in range(4)}))
    assert reader.poll() is True

    fake_pygame.joystick.unplug_all()

    assert reader.poll() is False


def test_read_twist_returns_zero_when_no_joystick():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)

    assert reader.read_twist() == (0.0, 0.0, 0.0)


def test_read_twist_converts_full_stick_deflection_with_correct_signs():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)
    # left stick fully up (forward) and fully right; right stick fully left
    fake_pygame.joystick.plug_in(0, FakeJoystick({0: 1.0, 1: -1.0, 2: 0.0, 3: -1.0}))
    reader.poll()

    linear_x, linear_y, angular_z = reader.read_twist()

    assert linear_x == MAX_LINEAR_SPEED  # stick forward -> +x
    assert linear_y == -MAX_LINEAR_SPEED  # stick right -> -y (ROS: +y is left)
    assert angular_z == MAX_ANGULAR_SPEED  # stick left -> +angular_z (counter-clockwise)


def test_init_restores_default_sigint_and_sigterm_handlers():
    # regression test: pygame's SDL backend installs its own SIGINT/SIGTERM
    # handlers as a side effect of joystick init, which silently breaks
    # Ctrl+C and process termination unless explicitly undone.
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # simulate SDL having clobbered it
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    GamepadReader(pygame_module=FakePygame())

    assert signal.getsignal(signal.SIGINT) is signal.default_int_handler
    assert signal.getsignal(signal.SIGTERM) == signal.SIG_DFL


def test_read_twist_applies_deadzone():
    fake_pygame = FakePygame()
    reader = GamepadReader(pygame_module=fake_pygame)
    # all axes within the deadzone - should read as exactly centered
    fake_pygame.joystick.plug_in(0, FakeJoystick({0: 0.05, 1: -0.05, 2: 0.0, 3: 0.08}))
    reader.poll()

    assert reader.read_twist() == (0.0, 0.0, 0.0)
