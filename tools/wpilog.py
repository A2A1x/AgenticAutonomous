"""Reads AdvantageKit .wpilog files, enough for sim benchmarks.

    python tools/wpilog.py logs/<file>.wpilog [key ...]

With no keys it lists the entries; with keys it prints each value as it changes.
Decodes doubles, ints, booleans, strings, their arrays, and Pose2d/Pose3d structs.
"""

import struct
import sys

DECODERS = {
    "double": lambda b: struct.unpack("<d", b)[0],
    "int64": lambda b: struct.unpack("<q", b)[0],
    "boolean": lambda b: b[0] != 0,
    "string": lambda b: b.decode("utf-8", "replace"),
    "struct:Pose2d": lambda b: struct.unpack("<3d", b),  # x, y, rotation rad
    "struct:Pose2d[]": lambda b: [struct.unpack_from("<3d", b, i) for i in range(0, len(b), 24)],
    "int64[]": lambda b: list(struct.unpack(f"<{len(b) // 8}q", b)),
    "string[]": lambda b: _strings(b),
    # x, y, z then the quaternion; only the translation is kept
    "struct:Pose3d[]": lambda b: [struct.unpack_from("<3d", b, i) for i in range(0, len(b), 56)],
}


def _strings(b):
    count, pos, out = struct.unpack_from("<I", b, 0)[0], 4, []
    for _ in range(count):
        n = struct.unpack_from("<I", b, pos)[0]
        out.append(b[pos + 4:pos + 4 + n].decode("utf-8", "replace"))
        pos += 4 + n
    return out


def read(path):
    """Returns {name: [(seconds, value), ...]} for every entry it can decode."""
    data = open(path, "rb").read()
    if data[:6] != b"WPILOG":
        raise ValueError(f"{path} is not a wpilog")
    pos = 12 + struct.unpack_from("<I", data, 8)[0]
    entries, out = {}, {}
    while pos < len(data):
        head = data[pos]
        id_len, size_len, ts_len = (head & 3) + 1, ((head >> 2) & 3) + 1, ((head >> 4) & 7) + 1
        pos += 1
        entry = int.from_bytes(data[pos:pos + id_len], "little"); pos += id_len
        size = int.from_bytes(data[pos:pos + size_len], "little"); pos += size_len
        ts = int.from_bytes(data[pos:pos + ts_len], "little"); pos += ts_len
        payload = data[pos:pos + size]; pos += size
        if pos > len(data):
            break  # a sim that exits mid-write leaves the last record cut off
        if entry == 0:
            if payload and payload[0] == 0:  # start: id, name, type, metadata
                eid = struct.unpack_from("<I", payload, 1)[0]
                n = struct.unpack_from("<I", payload, 5)[0]
                name = payload[9:9 + n].decode()
                t = struct.unpack_from("<I", payload, 9 + n)[0]
                typ = payload[13 + n:13 + n + t].decode()
                entries[eid] = (name, typ)
            continue
        name, typ = entries.get(entry, (None, None))
        decode = DECODERS.get(typ)
        if decode:
            out.setdefault(name, []).append((ts / 1e6, decode(payload)))
    return out


def at(series, t):
    """The latest value at or before time t, or None."""
    value = None
    for ts, v in series:
        if ts > t:
            break
        value = v
    return value


if __name__ == "__main__":
    log = read(sys.argv[1])
    if len(sys.argv) == 2:
        for name in sorted(log):
            print(name)
    for key in sys.argv[2:]:
        last = object()
        for ts, v in log.get(key, []):
            if v != last:
                print(f"{ts:8.2f} {key} {v}")
                last = v
