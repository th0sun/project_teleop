import json
import os
import sys

# Append paths to load TrajectoryRecorder directly
sys.path.append("/Users/thesun/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller")
sys.path.append("/Users/thesun/project_teleop_ws/project_teleop/src/dobot_mg400/mg400_controller/mg400_controller")

from common.trajectory.trajectory_recorder import TrajectoryRecorder

class MockLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warn(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")

def mock_send(cmd):
    # print(f"[ROBOT] {cmd}")
    pass

def test():
    logger = MockLogger()
    tr = TrajectoryRecorder(
        command_send_fn=mock_send,
        logger=logger,
        dashboard_send_fn=lambda x: None,
        get_position_fn=lambda: [0,0,0,0],
        waypoint_callback=lambda q: None
    )

    print("\n=== TEST 1: Receive Unity JSON Payload ===")
    payload = {
        "filename": "unity_mock_test.json",
        "frames": [
            {"timeStamp": 0.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0},
            {"timeStamp": 0.5, "j1": 10.0, "j2": 10.0, "j3": 10.0, "j4": 0.0},
            {"timeStamp": 1.0, "j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0}
        ]
    }
    json_str = json.dumps(payload)
    
    # Process just like vr_teleop_node would inside _traj_data_callback
    saved_path = tr.save_from_unity_json(json_str)
    
    print(f"Returned saved path: {saved_path}")
    expected_path = os.path.expanduser("~/project_teleop_ws/trajectories/unity_mock_test.json")
    if os.path.exists(expected_path):
        print("✅ SUCCESS: File saved correctly to disk.")
    else:
        print("❌ ERROR: File was not saved.")
        return

    print("\n=== TEST 2: Load the File ===")
    success = tr.load("unity_mock_test.json")
    if success and tr.loaded_name == "unity_mock_test.json":
        print("✅ SUCCESS: File loaded correctly into Memory.")
    else:
        print("❌ ERROR: Failed to load.")
        return
        
    print("\n=== TEST 3: Preview (Playback) ===")
    tr.start_preview()
    print("Preview thread started (simulating playback in background).")
    
    import time
    time.sleep(1.5)
    tr.stop_all()
    print("✅ SUCCESS: Preview initiated and stopped without crashing.")

if __name__ == "__main__":
    test()
