import struct

class TinyCodeReader:
    # The code reader has the I2C ID of hex 0c, or decimal 12.
    TINY_CODE_READER_I2C_ADDRESS = 0x0C

    # How long to pause between sensor polls.
    TINY_CODE_READER_DELAY = 0.05

    TINY_CODE_READER_LENGTH_OFFSET = 0
    TINY_CODE_READER_LENGTH_FORMAT = "H"
    TINY_CODE_READER_MESSAGE_OFFSET = TINY_CODE_READER_LENGTH_OFFSET + struct.calcsize(TINY_CODE_READER_LENGTH_FORMAT)
    TINY_CODE_READER_MESSAGE_SIZE = 254
    TINY_CODE_READER_MESSAGE_FORMAT = "B" * TINY_CODE_READER_MESSAGE_SIZE
    TINY_CODE_READER_I2C_FORMAT = TINY_CODE_READER_LENGTH_FORMAT + TINY_CODE_READER_MESSAGE_FORMAT
    TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(TINY_CODE_READER_I2C_FORMAT)

    def __init__(self, i2c_bus):
        self.i2c_bus = i2c_bus

    def poll(self):
        read_data = self.i2c_bus.readfrom(TinyCodeReader.TINY_CODE_READER_I2C_ADDRESS,
                                          TinyCodeReader.TINY_CODE_READER_I2C_BYTE_COUNT)

        message_length,  = struct.unpack_from(TinyCodeReader.TINY_CODE_READER_LENGTH_FORMAT,
                                              read_data,
                                              TinyCodeReader.TINY_CODE_READER_LENGTH_OFFSET)
        message_bytes = struct.unpack_from(TinyCodeReader.TINY_CODE_READER_MESSAGE_FORMAT,
                                           read_data,
                                           TinyCodeReader.TINY_CODE_READER_MESSAGE_OFFSET)

        if message_length == 0:
            return None

        try:
            message_string = bytearray(message_bytes[0:message_length]).decode("utf-8")
            return message_string
        except:
            print(f"TinyCodeReader: Couldn't decode as UTF 8.  Data: {read_data}")
            return None
