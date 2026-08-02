"""Wire-framing primitives shared by the AA-framed protocols and the serial link."""


def calculate_checksum(frame: bytes) -> int:
    """LRC checksum: two's complement of the low byte of the sum."""
    return (0x100 - (sum(frame) & 0xFF)) & 0xFF
