source /opt/ros/humble/setup.bash
if [ -f /opt/ros_ws/install/setup.bash ]; then
  source /opt/ros_ws/install/setup.bash
fi
export PYTHONPATH=/opt/carla/PythonAPI/carla:/opt/scenario_runner:${PYTHONPATH:-}
export SCENARIO_RUNNER_ROOT=/opt/scenario_runner
