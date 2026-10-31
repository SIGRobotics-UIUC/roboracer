"""Optional bounded asynchronous raw-frame trace for simulator timing experiments."""
import json
import queue
import threading


class FrameTrace:
    def __init__(self, path):
        self.stream = open(path, 'x')  # never overwrite a previous experiment
        self.pending = queue.Queue(maxsize=2048)
        self.dropped = 0
        self.error = None
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def add(self, record):
        try:
            self.pending.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            with self.stream:
                while True:
                    record = self.pending.get()
                    if record is None:
                        break
                    self.stream.write(json.dumps(record) + '\n')
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        while self.worker.is_alive():
            try:
                self.pending.put(None, timeout=.1)
                break
            except queue.Full:
                continue
        self.worker.join(timeout=5)
        return dict(dropped=self.dropped, error=self.error, writer_alive=self.worker.is_alive())


def compact_payload(payload):
    """Keep sensor values and all field names; omit bulky images and lidar payloads."""
    return {key: value for key, value in payload.items()
            if 'image' not in key.lower() and 'range array' not in key.lower()}
