"""Dynamixel Protocol 2.0 instruction opcodes.

Just the opcodes the driver uses. Add more only when a caller needs them.
"""

PING = 0x01
READ = 0x02
WRITE = 0x03
SYNC_READ = 0x82
SYNC_WRITE = 0x83
