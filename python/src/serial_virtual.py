import re
import subprocess


class VirtualSerialPair:
    def __init__(self):
        self.proc = subprocess.Popen(
            ["socat", "-d", "-d", "pty,raw,echo=0", "pty,raw,echo=0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        ports = []
        pattern = re.compile(r"PTY is (\S+)")

        while True:
            line = self.proc.stderr.readline()
            if not line:
                break

            match = pattern.search(line)
            if match:
                port_path = match.group(1)
                ports.append(port_path)

            if len(ports) == 2:
                break

        self.port1 = ports[0]
        self.port2 = ports[1]

    def __del__(self):
        self.proc.kill()
