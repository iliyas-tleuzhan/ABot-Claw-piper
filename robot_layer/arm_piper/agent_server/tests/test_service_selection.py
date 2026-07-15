#!/usr/bin/env python3
"""Read-only tests for perception HTTP health classification."""

from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from robot_sdk import service_selection


class _Response:
    def __init__(self, body):
        self._body = body

    def raise_for_status(self):
        return None

    def json(self):
        return self._body


class ServiceSelectionTests(unittest.TestCase):
    def test_ok_health(self):
        with mock.patch.object(service_selection.requests, "get", return_value=_Response({"status": "ok"})):
            status, body = service_selection.service_health_status("http://127.0.0.1:8013/detect")
        self.assertEqual(status, "ok")
        self.assertEqual(body["status"], "ok")

    def test_degraded_health(self):
        with mock.patch.object(service_selection.requests, "get", return_value=_Response({"status": "degraded"})):
            status, body = service_selection.service_health_status("http://127.0.0.1:8015/grasp/detect")
        self.assertEqual(status, "degraded")
        self.assertEqual(body["status"], "degraded")

    def test_unreachable_health(self):
        with mock.patch.object(service_selection.requests, "get", side_effect=OSError("refused")):
            status, body = service_selection.service_health_status("http://127.0.0.1:8013/detect")
        self.assertEqual(status, "unavailable")
        self.assertIn("error", body)


if __name__ == "__main__":
    unittest.main()
