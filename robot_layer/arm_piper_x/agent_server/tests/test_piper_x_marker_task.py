import json
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
PARSER = ROOT / "openclaw_layer/skills/abotclaw-piper-x-manipulation/scripts/piper_x_marker_task.py"


class PiperXMarkerParserTest(unittest.TestCase):
    def parse(self, text):
        output = subprocess.check_output(
            [sys.executable, str(PARSER), text, "--plan-only"],
            text=True,
        )
        return json.loads(output)

    def test_touch_marker_routes_to_piper_x_runner(self):
        parsed = self.parse("touch the aruco marker")
        self.assertEqual(parsed["action"], "touch")
        self.assertEqual(parsed["selected_robot"], "PiPER-X")
        self.assertIn("robot_layer/arm_piper_x/agent_server/run_piper_x_agent_task.py", parsed["command_text"])
        self.assertIn("touch", parsed["command_text"])
        self.assertIn("--plan-only", parsed["command_text"])

    def test_approach_marker_routes_to_approach(self):
        parsed = self.parse("approach the marker")
        self.assertEqual(parsed["action"], "approach")
        self.assertIn("approach", parsed["command_text"])

    def test_home_routes_to_home_endpoint(self):
        parsed = self.parse("go to home pose")
        self.assertEqual(parsed["action"], "home")
        self.assertIn(" home ", parsed["command_text"])

    def test_save_home_routes_to_non_motion_snapshot(self):
        parsed = self.parse("save current pose as home")
        self.assertEqual(parsed["action"], "save-home")
        self.assertIn("save-home", parsed["command_text"])

    def test_open_gripper_routes_to_gripper_tool(self):
        parsed = self.parse("open the gripper")
        self.assertEqual(parsed["action"], "open-gripper")
        self.assertIn("open-gripper", parsed["command_text"])
        self.assertIn("--plan-only", parsed["command_text"])

    def test_close_gripper_routes_to_gripper_tool(self):
        parsed = self.parse("close the claw")
        self.assertEqual(parsed["action"], "close-gripper")
        self.assertIn("close-gripper", parsed["command_text"])
        self.assertEqual(parsed["completion_type"], "gripper_width_command")


if __name__ == "__main__":
    unittest.main()
