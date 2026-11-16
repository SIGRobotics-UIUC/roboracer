"""Owned process groups; never signal unrelated ROS processes."""
import os
import signal
import subprocess
import time


class Process:
    def __init__(self, command, log, env=None):
        self.stream = open(log, 'x')
        try:
            self.proc = subprocess.Popen(command, stdout=self.stream, stderr=subprocess.STDOUT,
                                         start_new_session=True, env=env)
        except BaseException:
            self.stream.close()
            raise
        self.command = command

    def alive(self):
        return self.proc.poll() is None

    def signal(self, sig=signal.SIGINT):
        try:
            os.killpg(self.proc.pid, sig)
        except ProcessLookupError:
            pass

    def group_alive(self):
        self.proc.poll()
        try:
            os.killpg(self.proc.pid, 0)
            return True
        except ProcessLookupError:
            return False

    def close(self, spin=lambda seconds: time.sleep(seconds)):
        escalations = []
        for sig, grace in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            if not self.group_alive():
                break
            self.signal(sig)
            if sig != signal.SIGINT:
                escalations.append(sig.name)
            deadline = time.monotonic() + grace
            while self.group_alive() and time.monotonic() < deadline:
                spin(0.05)
        self.stream.close()
        return {'exit_code': self.proc.poll(), 'escalations': escalations,
                'group_alive': self.group_alive()}
