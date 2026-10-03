"""Process isolation and bounded lifecycle checks for shadow diagnostics."""

import queue
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'ros2', 'm3pro_nav'))

from m3pro_nav.frame_grid_shadow_worker import FrameGridShadowWorker


class _FakeQueue:
    def __init__(self, *, full=False):
        self.full = full
        self.items = []
        self.closed = False

    def put_nowait(self, value):
        if self.full:
            raise queue.Full
        self.items.append(value)

    def get_nowait(self):
        if not self.items:
            raise queue.Empty
        return self.items.pop(0)

    def cancel_join_thread(self):
        pass

    def close(self):
        self.closed = True


class _FakeProcess:
    def __init__(self):
        self.alive = False
        self.terminated = False
        self.join_calls = []

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        self.join_calls.append(timeout)

    def terminate(self):
        self.terminated = True
        self.alive = False


class _FakeContext:
    def __init__(self):
        self.queues = []
        self.process = _FakeProcess()

    def Queue(self, maxsize):
        q = _FakeQueue()
        self.queues.append((maxsize, q))
        return q

    def Process(self, **_kwargs):
        return self.process


def test_worker_submission_uses_nonblocking_bounded_queue_and_shutdown():
    ctx = _FakeContext()
    worker = FrameGridShadowWorker(queue_size=2, mp_context=ctx)
    requests = ctx.queues[0][1]
    requests.full = True

    assert worker.submit(1.0, object(), object(), object(), object()) is False
    assert ctx.process.alive
    assert requests.items == []

    # A full queue must not make shutdown wait for work to drain.
    worker.shutdown(timeout_s=0.01)
    assert ctx.process.terminated
    assert ctx.process.join_calls == [0.01, 0.01]
    assert all(q.closed for _, q in ctx.queues)


def test_worker_poll_is_nonblocking_and_returns_completed_records():
    ctx = _FakeContext()
    worker = FrameGridShadowWorker(mp_context=ctx)
    results = ctx.queues[1][1]
    results.items.append((1.25, {'accepted': False}, None))
    assert worker.poll() == [(1.25, {'accepted': False}, None)]
    assert worker.poll() == []
    worker.shutdown()


def test_worker_crash_reports_accepted_but_unfinished_job():
    ctx = _FakeContext()
    worker = FrameGridShadowWorker(mp_context=ctx)
    assert worker.submit(2.0, object(), None, None, None)
    ctx.process.alive = False
    assert worker.poll() == [(
        2.0, None, 'WorkerProcessExited before producing a result')]
    worker.shutdown()


def test_worker_start_failure_is_reported_and_resources_close():
    ctx = _FakeContext()

    def fail_start():
        raise RuntimeError('spawn denied')

    ctx.process.start = fail_start
    worker = FrameGridShadowWorker(mp_context=ctx)
    assert not worker.start()
    assert worker.startup_error == 'RuntimeError: spawn denied'
    worker.shutdown()
    assert all(q.closed for _, q in ctx.queues)


def test_worker_executes_diagnostic_in_child_process():
    worker = FrameGridShadowWorker(queue_size=1)
    try:
        # The malformed frame returns a diagnostic error quickly; importantly,
        # processing happens in the child and cannot run on the caller thread.
        assert worker.submit(1.25, object(), None, None, None)
        assert worker._process.pid != os.getpid()
        deadline = time.monotonic() + 3.0
        completed = []
        while not completed and time.monotonic() < deadline:
            completed = worker.poll()
            if not completed:
                time.sleep(0.01)
        assert len(completed) == 1
        stamp, result, error = completed[0]
        assert stamp == 1.25 and result is None
        assert error.startswith('AttributeError:')
    finally:
        worker.shutdown()
