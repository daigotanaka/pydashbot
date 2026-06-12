"""Transport-independent robot command request handling."""

import json


def create_request(method, args=(), kwargs=None):
    return {"method": method, "args": list(args), "kwargs": kwargs or {}}


def execute_request(request, robot):
    """Execute a decoded command request and return a response dictionary."""
    try:
        method = request["method"]
        args = request.get("args", [])
        kwargs = request.get("kwargs", {})
        result = getattr(robot, method)(*args, **kwargs)
        return {"ok": True, "result": result}
    except Exception as error:
        return {"ok": False, "error": str(error)}


def execute_json(message, robot):
    try:
        return execute_request(json.loads(message), robot)
    except Exception as error:
        return {"ok": False, "error": str(error)}


def parse_cli_values(values):
    parsed = []
    for value in values:
        try:
            parsed.append(json.loads(value))
        except json.JSONDecodeError:
            parsed.append(value)
    return parsed
