import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
INDEX_FILE = ROOT / "web_panel" / "templates" / "index.html"


class WebPanelTemplateTest(unittest.TestCase):
    def test_line_follow_controls_are_hidden_while_shelved(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertNotIn('<h2>视觉巡线</h2>', text)
        self.assertNotIn('启用巡线控制', text)
        self.assertNotIn('仅更新预览', text)
        self.assertNotIn('/api/line_follow/status', text)
        self.assertNotIn('/api/line_follow/debug.jpg', text)
        self.assertNotIn('setInterval(refreshLineFollow', text)
        self.assertIn('视觉巡线功能已暂时隐藏', text)

    def test_voice_control_panel_is_available(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertIn("语音控制", text)
        self.assertIn("startVoiceRecognition", text)
        self.assertIn("sendVoiceCommand", text)
        self.assertIn("/api/voice_command", text)
        self.assertIn("SpeechRecognition", text)

    def test_voice_control_uses_start_and_stop_buttons(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertIn("开始录音", text)
        self.assertIn("结束录音并执行", text)
        self.assertIn("startVoiceRecording", text)
        self.assertIn("stopVoiceRecording", text)
        self.assertIn("voiceListening", text)
        self.assertIn("点“结束录音并执行”", text)
        self.assertIn("没有收到语音结果", text)
        self.assertIn("checkVoiceMicrophone", text)
        self.assertIn("navigator.mediaDevices.getUserMedia", text)
        self.assertIn("micLevel", text)

    def test_voice_control_does_not_use_pointer_hold_events(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertNotIn("onpointerdown", text)
        self.assertNotIn("onpointerup", text)
        self.assertNotIn("setPointerCapture", text)
        self.assertNotIn("beginVoiceHold", text)
        self.assertNotIn("finishVoiceHold", text)
        self.assertIn("rec.continuous=false", text)

    def test_camera_card_appears_before_safety_card(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertLess(text.index("<h2>Camera</h2>"), text.index("<h2>Safety</h2>"))

    def test_patrol_can_use_current_robot_pose_as_candidate(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertIn("使用当前地图目标", text)
        self.assertIn("使用小车位置", text)
        self.assertIn("useCurrentPoseForPatrol", text)
        self.assertIn("latestStatus.pose", text)

    def test_person_follow_controls_are_available(self):
        text = INDEX_FILE.read_text(encoding="utf-8")

        self.assertIn("人体跟随", text)
        self.assertIn("startPersonFollow", text)
        self.assertIn("stopPersonFollow", text)
        self.assertIn("/api/person_follow/start", text)
        self.assertIn("/api/person_follow/stop", text)
        self.assertIn("personFollowStatus", text)
        start_block = text.split("async function startPersonFollow()", 1)[1].split("async function stopPersonFollow()", 1)[0]
        self.assertIn("stopCameraLive()", start_block)


if __name__ == "__main__":
    unittest.main()
