import re
import subprocess
import threading


class VirtualSerialPair:
    def __init__(self):
        pass

    def __enter__(self):
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

        self.stop = False

        self.thread = threading.Thread(target=self.drain)
        self.thread.start()

        return (self.port1, self.port2)

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop = True
        self.proc.kill()
        self.thread.join()

    def drain(self):
        while not self.stop:
            self.proc.stderr.read(64)
