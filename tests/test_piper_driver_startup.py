import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]


class PiperDriverStartupTests(unittest.TestCase):
    def test_lower_launcher_sets_both_gripper_param_spellings(self) -> None:
        script = (REPO_ROOT / "start_abotclaw_all.sh").read_text()
        self.assertIn("_gripper_exist:=true", script)
        self.assertIn("_girpper_exist:=true", script)

    def test_lower_launcher_repairs_missing_or_stale_driver(self) -> None:
        script = (REPO_ROOT / "start_abotclaw_all.sh").read_text()
        self.assertIn("repair_existing_session()", script)
        self.assertIn("restart_piper_driver_window()", script)
        self.assertIn("pkill -f '[p]iper_ctrl_single_node.py' || true", script)

    def test_launch_files_set_driver_consumed_param(self) -> None:
        for relative_path in (
            "robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros/src/piper/launch/start_single_piper.launch",
            "robot_layer/arm_piper/agent_server/robot_driver_ros/src/piper_ros/src/piper/launch/start_single_piper_rviz.launch",
        ):
            launch_file = (REPO_ROOT / relative_path).read_text()
            self.assertIn('<param name="gripper_exist" value="true" />', launch_file)
            self.assertIn('<param name="girpper_exist" value="true" />', launch_file)

    def test_trajectory_bridge_uses_timed_resampling_and_short_final_settle(self) -> None:
        bridge = (REPO_ROOT / "robot_layer/arm_piper/agent_server/piper_trajectory_bridge.py").read_text()
        self.assertIn('trajectory_command_rate_hz", 50.0', bridge)
        self.assertIn('final_settle_timeout_s", 0.5', bridge)
        self.assertIn("def build_command_points(self, points):", bridge)
        self.assertIn("np.interp(", bridge)


if __name__ == "__main__":
    unittest.main()
