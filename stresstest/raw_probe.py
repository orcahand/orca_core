import serial, time, collections, sys

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"
for baud in (2000000, 1000000, 921600, 460800, 115200):
    try:
        s = serial.Serial(port, baudrate=baud, timeout=0.2)
    except Exception as e:
        print(baud, "open err", e)
        continue
    time.sleep(0.2)
    s.reset_input_buffer()
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < 0.8:
        d = s.read(4096)
        if d:
            buf += d
    s.close()
    follow = collections.Counter()
    for i in range(len(buf) - 1):
        if buf[i] == 0xAA:
            follow[buf[i + 1]] += 1
    top = ", ".join("0x%02x:%d" % (k, v) for k, v in follow.most_common(6))
    print("baud=%8d bytes=%6d  AA-follows={%s}  head=%s"
          % (baud, len(buf), top, buf[:20].hex()))
