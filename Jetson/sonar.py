from smbus2 import SMBus, i2c_msg


class Sonar:
    I2C_ADDR = 0x77
    BUS_ID = 1
    DISTANCE_REG = 0
    MAX_DISTANCE_MM = 5000

    RGB_MODE_REG = 2
    COLOR_REGS = (3, 6)
    BREATH_REGS = (9, 12)

    def __init__(self, i2c_addr=I2C_ADDR, bus_id=BUS_ID):
        self.i2c_addr = i2c_addr
        self.bus_id = bus_id
        self.pixels = [0, 0]

    def _write_byte(self, register: int, value: int) -> bool:
        try:
            with SMBus(self.bus_id) as bus:
                bus.write_byte_data(self.i2c_addr, register, value & 0xFF)
        except OSError:
            return False

        return True

    def set_rgb_mode(self, mode: int) -> bool:
        return self._write_byte(self.RGB_MODE_REG, int(mode))

    def set_color(self, index: int, rgb: int) -> bool:
        self._require_pixel(index)
        rgb = int(rgb)
        start_reg = self.COLOR_REGS[index]
        values = ((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF)

        writes_ok = all(
            self._write_byte(start_reg + offset, value)
            for offset, value in enumerate(values)
        )

        if writes_ok:
            self.pixels[index] = rgb
            return True

        return False

    def get_color(self, index: int):
        self._require_pixel(index)
        rgb = self.pixels[index]
        return (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF

    def set_breath_cycle(self, index: int, rgb_channel: int, cycle_ms: int) -> bool:
        self._require_pixel(index)

        if rgb_channel not in {0, 1, 2}:
            raise ValueError("rgb_channel must be 0, 1, or 2")

        register = self.BREATH_REGS[index] + rgb_channel
        return self._write_byte(register, int(cycle_ms / 100))

    def start_symphony(self):
        self.set_rgb_mode(1)
        self.set_breath_cycle(1, 0, 2000)
        self.set_breath_cycle(1, 1, 3300)
        self.set_breath_cycle(1, 2, 4700)
        self.set_breath_cycle(0, 0, 4600)
        self.set_breath_cycle(0, 1, 2000)
        self.set_breath_cycle(0, 2, 3400)

    def get_distance(self) -> int:
        try:
            with SMBus(self.bus_id) as bus:
                bus.i2c_rdwr(i2c_msg.write(self.i2c_addr, [self.DISTANCE_REG]))
                read = i2c_msg.read(self.i2c_addr, 2)
                bus.i2c_rdwr(read)
        except OSError:
            return self.MAX_DISTANCE_MM

        distance = int.from_bytes(bytes(list(read)), byteorder="little", signed=False)
        return min(distance, self.MAX_DISTANCE_MM)

    @staticmethod
    def _require_pixel(index: int):
        if index not in {0, 1}:
            raise ValueError("index must be 0 or 1")
