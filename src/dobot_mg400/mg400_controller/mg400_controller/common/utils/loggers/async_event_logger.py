#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Reusable background event logger.

This keeps disk I/O and non-critical analytics work out of real-time ROS
callbacks and control loops.
"""

import csv
import queue
import threading


class AsyncEventLogger:
    def __init__(self, csv_path=None, csv_header=None, handlers=None, error_handler=None):
        self._queue = queue.Queue()
        self._stop_event = threading.Event()
        self._handlers = handlers or {}
        self._error_handler = error_handler or self._default_error_handler
        self._csv_file = None
        self._csv_writer = None
        self.csv_path = csv_path

        if csv_path:
            self._csv_file = open(csv_path, mode="w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            if csv_header:
                self._csv_writer.writerow(csv_header)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def put(self, event_name, payload):
        self._queue.put((event_name, payload))

    def close(self, timeout=1.0):
        self._stop_event.set()
        self._thread.join(timeout=timeout)
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()

    def _run(self):
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                try:
                    event_name, payload = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if event_name == "CSV":
                    self._write_csv(payload)
                else:
                    handler = self._handlers.get(event_name)
                    if handler:
                        handler(*payload)

                self._queue.task_done()
            except Exception as exc:
                self._error_handler(exc)

    def _write_csv(self, row):
        if not self._csv_writer:
            return
        self._csv_writer.writerow(row)
        if self._queue.qsize() == 0:
            self._csv_file.flush()

    @staticmethod
    def _default_error_handler(exc):
        print(f"Error in AsyncEventLogger: {exc}")
