"""Bounded subprocess runner for the optional frame-grid shadow diagnostic.

The worker owns its estimator recurrence. The navigation process only sends
immutable snapshots and polls completed diagnostic results; no scan fitting
or solver work runs on the ROS executor thread.
"""

from __future__ import annotations

import multiprocessing as mp
import queue


def _worker_main(requests, results):
    from .frame_grid_shadow import FrameGridShadow

    estimator = FrameGridShadow()
    while True:
        item = requests.get()
        if item is None:
            return
        stamp, frame, pose, extrinsic, anchor = item
        try:
            result = estimator.process(frame, pose, extrinsic, anchor)
            results.put((stamp, result, None))
        except Exception as exc:  # diagnostic failures remain diagnostic
            results.put((stamp, None, f'{type(exc).__name__}: {exc}'))


class FrameGridShadowWorker:
    """One persistent isolated worker with bounded input and shutdown."""

    def __init__(self, *, queue_size=2, mp_context=None):
        if queue_size < 1:
            raise ValueError('queue_size must be positive')
        self._ctx = mp_context or mp.get_context('spawn')
        self._requests = self._ctx.Queue(maxsize=queue_size)
        # Result volume is bounded by accepted input plus the single active job.
        self._results = self._ctx.Queue(maxsize=queue_size + 1)
        self._process = self._ctx.Process(
            target=_worker_main, args=(self._requests, self._results),
            name='frame-grid-shadow', daemon=True)
        self._started = False
        self._closed = False
        self._resources_closed = False
        self._startup_error = None
        self._inflight = set()

    def start(self):
        """Start outside executor callbacks; repeated calls are harmless."""
        if self._started:
            return True
        if self._closed:
            return False
        try:
            self._process.start()
            self._started = True
            return True
        except Exception as exc:  # diagnostics must not fault navigation
            self._startup_error = f'{type(exc).__name__}: {exc}'
            self._closed = True
            return False

    @property
    def startup_error(self):
        return self._startup_error

    @property
    def started(self):
        return self._started

    def submit(self, stamp, frame, pose, extrinsic, anchor, *,
               start_if_needed=True):
        if self._closed:
            return False
        if not self._started:
            if not start_if_needed or not self.start():
                return False
        if not self._process.is_alive():
            return False
        try:
            self._requests.put_nowait((stamp, frame, pose, extrinsic, anchor))
            self._inflight.add(stamp)
            return True
        except queue.Full:
            return False

    def poll(self, limit=8):
        completed = []
        for _ in range(limit):
            try:
                record = self._results.get_nowait()
                completed.append(record)
                self._inflight.discard(record[0])
            except queue.Empty:
                break
            except (EOFError, OSError, ValueError):
                break
        if self._started and not self._process.is_alive():
            completed.extend(
                (stamp, None, 'WorkerProcessExited before producing a result')
                for stamp in sorted(self._inflight))
            self._inflight.clear()
        return completed

    def shutdown(self, timeout_s=1.0):
        if self._resources_closed:
            return []
        self._closed = True
        if self._started:
            try:
                if self._process.is_alive():
                    try:
                        self._requests.put_nowait(None)
                    except queue.Full:
                        pass
                self._process.join(timeout_s)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout_s)
            except (AssertionError, OSError, ValueError):
                pass
        # Read final child output while the result queue is still open.
        completed = self.poll()
        for channel in (self._requests, self._results):
            channel.cancel_join_thread()
            channel.close()
        self._resources_closed = True
        # After join/termination, account for every accepted request. Preserve
        # completed results for the caller and mark only unresolved jobs lost.
        unresolved = [(stamp, None,
                       'WorkerShutdown before producing a result')
                      for stamp in sorted(self._inflight)]
        self._inflight.clear()
        return completed + unresolved
