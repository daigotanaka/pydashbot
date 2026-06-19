"""Standalone live map dashboard server.

Runs independently of the mapping app. Start it first
(``python -m apps.dashboard``), then run a mapping session with
``dashboard.active: true`` and the mapper will POST its poses to ``/move``.
"""
