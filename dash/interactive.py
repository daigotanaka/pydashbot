"""Synchronous facade for controlling a robot from an interactive prompt."""

import asyncio
import inspect
import threading

from dash.robot import DEFAULT_ROBOT_NAME, discover_and_connect


class InteractiveRobot:
    """Run a connected async Robot on a background event loop.

    Coroutine methods on the wrapped robot are exposed as blocking synchronous
    methods, which makes the robot convenient to use from a standard Python
    REPL or notebook.
    """

    def __init__(self, robot, loop, thread):
        self._robot = robot
        self._loop = loop
        self._thread = thread
        self._closed = False

    @property
    def async_robot(self):
        """Return the underlying asynchronous Robot instance."""
        return self._robot

    def __getattr__(self, name):
        attribute = getattr(self._robot, name)
        if not inspect.iscoroutinefunction(attribute):
            return attribute

        def call(*args, **kwargs):
            return self._run(self._call_method(name, args, kwargs))

        return call

    async def _call_method(self, name, args, kwargs):
        return await getattr(self._robot, name)(*args, **kwargs)

    def _run(self, coroutine):
        if self._closed:
            coroutine.close()
            raise RuntimeError("Robot connection is closed")
        future = asyncio.run_coroutine_threadsafe(coroutine, self._loop)
        return future.result()

    def disconnect(self):
        """Disconnect from the robot and stop its background event loop."""
        if self._closed:
            return

        try:
            self._run(self._robot.disconnect())
        finally:
            self._closed = True
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._thread.join()

    close = disconnect

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()


def _run_event_loop(loop, ready):
    asyncio.set_event_loop(loop)
    ready.set()
    loop.run_forever()

    pending = asyncio.all_tasks(loop)
    for task in pending:
        task.cancel()
    if pending:
        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
    loop.close()


def discover_and_connect_sync(
    retry_attempts=3,
    retry_delay=5,
    name=DEFAULT_ROBOT_NAME,
    address=None,
):
    """Connect to Dash and return a synchronous interactive handle."""
    loop = asyncio.new_event_loop()
    ready = threading.Event()
    thread = threading.Thread(
        target=_run_event_loop,
        args=(loop, ready),
        name="pydashbot-event-loop",
        daemon=True,
    )
    thread.start()
    ready.wait()

    future = asyncio.run_coroutine_threadsafe(
        discover_and_connect(
            retry_attempts=retry_attempts,
            retry_delay=retry_delay,
            name=name,
            address=address,
        ),
        loop,
    )
    try:
        robot = future.result()
    except BaseException:
        loop.call_soon_threadsafe(loop.stop)
        thread.join()
        raise

    if robot is None:
        loop.call_soon_threadsafe(loop.stop)
        thread.join()
        return None

    return InteractiveRobot(robot, loop, thread)
