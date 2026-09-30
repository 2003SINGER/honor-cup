"""ROS-free checks that scan_debug launch preserves string parameter types."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import yaml


class LaunchTypeTest(unittest.TestCase):
    def test_string_launch_values_are_explicitly_typed(self):
        class LaunchDescription:
            def __init__(self, entities):
                self.entities = entities

        class DeclareLaunchArgument:
            def __init__(self, name, **kwargs):
                self.name = name
                self.kwargs = kwargs

        class LaunchConfiguration:
            def __init__(self, name):
                self.name = name

        class PythonExpression:
            def __init__(self, expression):
                self.expression = expression

        class ParameterValue:
            def __init__(self, value, *, value_type):
                self.value = value
                self.value_type = value_type

        class Node:
            def __init__(self, **kwargs):
                self.kwargs = kwargs

        launch = types.ModuleType('launch')
        launch.LaunchDescription = LaunchDescription
        actions = types.ModuleType('launch.actions')
        actions.DeclareLaunchArgument = DeclareLaunchArgument
        substitutions = types.ModuleType('launch.substitutions')
        substitutions.LaunchConfiguration = LaunchConfiguration
        substitutions.PythonExpression = PythonExpression
        launch_ros = types.ModuleType('launch_ros')
        ros_actions = types.ModuleType('launch_ros.actions')
        ros_actions.Node = Node
        descriptions = types.ModuleType('launch_ros.parameter_descriptions')
        descriptions.ParameterValue = ParameterValue
        modules = {
            'launch': launch,
            'launch.actions': actions,
            'launch.substitutions': substitutions,
            'launch_ros': launch_ros,
            'launch_ros.actions': ros_actions,
            'launch_ros.parameter_descriptions': descriptions,
        }
        path = (Path(__file__).parents[1] / 'ros2/m3pro_nav/launch'
                / 'scan_debug.launch.py')
        spec = importlib.util.spec_from_file_location('scan_debug_launch_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
            description = module.generate_launch_description()

        parameters = description.entities[-1].kwargs['parameters'][0]
        for name in ('heading', 'scan_topic', 'odom_topic', 'imu_topic',
                     'expected_laser_frame', 'expected_odom_frame',
                     'expected_base_frame', 'laser_extrinsic_yaml', 'session_dir'):
            with self.subTest(parameter=name):
                self.assertIsInstance(parameters[name], ParameterValue)
                self.assertIs(parameters[name].value_type, str)
        heading_value = parameters['heading'].value
        self.assertIsInstance(heading_value, PythonExpression)
        for letter, word in module.HEADING_WORDS.items():
            expression = ''.join(
                letter if isinstance(part, LaunchConfiguration) else part
                for part in heading_value.expression
            )
            self.assertEqual(eval(expression), word)
            emitted_yaml = yaml.dump({
                '/scan_debug': {'ros__parameters': {'heading': word}},
            })
            loaded = yaml.safe_load(emitted_yaml)
            loaded_heading = loaded['/scan_debug']['ros__parameters']['heading']
            self.assertIsInstance(loaded_heading, str)
            self.assertEqual(loaded_heading, word)
        self.assertEqual(parameters['expected_base_frame'].value.name,
                         'expected_base_frame')
        self.assertIsInstance(parameters['use_tf_extrinsic'], LaunchConfiguration)


if __name__ == '__main__':
    unittest.main()
