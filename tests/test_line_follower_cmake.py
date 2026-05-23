import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CMAKE_FILE = ROOT / "CMakeLists.txt"
PACKAGE_FILE = ROOT / "package.xml"
CPP_FILE = ROOT / "src" / "line_follower.cpp"


class LineFollowerCMakeTest(unittest.TestCase):
    def test_catkin_cmake_builds_line_follower_node_with_required_dependencies(self):
        text = CMAKE_FILE.read_text(encoding="utf-8")

        self.assertIn("project(roscar1)", text)
        self.assertIn("find_package(catkin REQUIRED COMPONENTS", text)
        for dependency in (
            "roscpp",
            "cv_bridge",
            "image_transport",
            "sensor_msgs",
            "geometry_msgs",
            "std_msgs",
        ):
            self.assertRegex(text, rf"\b{dependency}\b")

        self.assertIn("find_package(OpenCV REQUIRED)", text)
        self.assertRegex(
            text,
            r"add_executable\s*\(\s*line_follower_node\s+src/line_follower\.cpp\s*\)",
        )
        self.assertIn("${catkin_LIBRARIES}", text)
        self.assertIn("${OpenCV_LIBRARIES}", text)

    def test_package_xml_declares_runtime_and_build_dependencies(self):
        text = PACKAGE_FILE.read_text(encoding="utf-8")

        self.assertIn("<name>roscar1</name>", text)
        self.assertIn("<buildtool_depend>catkin</buildtool_depend>", text)
        for dependency in (
            "roscpp",
            "cv_bridge",
            "image_transport",
            "sensor_msgs",
            "geometry_msgs",
            "std_msgs",
            "opencv3",
        ):
            self.assertRegex(text, rf"<build_depend>{dependency}</build_depend>")
            self.assertRegex(text, rf"<exec_depend>{dependency}</exec_depend>")

    def test_cpp_source_lives_under_src_for_catkin_target(self):
        self.assertTrue(CPP_FILE.exists())
        self.assertIn("class LineFollower", CPP_FILE.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
