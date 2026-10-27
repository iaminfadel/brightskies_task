#!/usr/bin/env python3
"""Contract tests for the scenario harness (#37).

Run inside the deps container (pytest is in the image):
    docker run --rm -v /ws:/ws ... python3 -m pytest /ws/brightdrive/scripts/scenario/tests -q
Covers the pieces that must never drift: the GNSS profile shared by the
scenario relay and the estimator config, the ScenarioRunner pin, the .xosc
scenario contracts, and the coverage-check coordinate chain.
"""
import re
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / "src/brightdrive_bringup/scripts"))
import check_route_coverage  # noqa: E402

SCENARIOS = sorted((REPO / "scenarios").glob("*.xosc"))

PROFILES_FILE = REPO / "src/brightdrive_bringup/config/gnss_profiles.yaml"
LOCALIZATION_FILE = REPO / "src/brightdrive_bringup/config/localization.yaml"
PIN_FILE = REPO / "config/scenario/scenario_runner.yaml"


# ------------------------------------------------------------ GNSS profiles


def test_profiles_have_required_keys():
    doc = yaml.safe_load(PROFILES_FILE.read_text())
    assert doc["degrees_per_metre"] > 0
    for name in ("nominal", "noisy"):
        profile = doc["profiles"][name]
        for key in ("sigma_h_m", "sigma_v_m", "bias_east_m", "bias_north_m",
                    "dropout_windows_s", "seed"):
            assert key in profile, f"{name}.{key} missing"
    noisy = doc["profiles"]["noisy"]
    assert noisy["sigma_h_m"] > doc["profiles"]["nominal"]["sigma_h_m"]
    assert len(noisy["dropout_windows_s"]) >= 1


def test_estimator_config_matches_nominal_profile():
    """The single-source guarantee (#37): the checked-in estimator GNSS
    sigmas must equal the nominal profile, since launch overrides them from
    the profile at run time and the checked-in defaults must not disagree."""
    profile = yaml.safe_load(PROFILES_FILE.read_text())["profiles"]["nominal"]
    doc = yaml.safe_load(LOCALIZATION_FILE.read_text())
    gnss = doc["localization_core"]["ros__parameters"]["gnss"]
    assert gnss["sigma_xy_m"] == pytest.approx(profile["sigma_h_m"])
    assert gnss["sigma_z_m"] == pytest.approx(profile["sigma_v_m"])
    # The estimator consumes the relay's output, not raw GNSS.
    assert doc["localization_core"]["ros__parameters"]["topics"]["gnss"] == "/localization/gnss"


def test_relay_profile_loader_and_degradation():
    """Import the relay without a ROS runtime and exercise its math."""
    rclpy_stub = type(sys)("rclpy")
    node_stub = type(sys)("rclpy.node")
    qos_stub = type(sys)("rclpy.qos")
    sensor_stub = type(sys)("sensor_msgs")
    sensor_msgs_msg = type(sys)("sensor_msgs.msg")
    ament_stub = type(sys)("ament_index_python")
    packages_stub = type(sys)("ament_index_python.packages")
    for name, mod in [("rclpy", rclpy_stub), ("rclpy.node", node_stub),
                      ("rclpy.qos", qos_stub), ("sensor_msgs", sensor_stub),
                      ("sensor_msgs.msg", sensor_msgs_msg),
                      ("ament_index_python", ament_stub),
                      ("ament_index_python.packages", packages_stub)]:
        sys.modules.setdefault(name, mod)
    node_stub.Node = type("Node", (), {})
    qos_stub.qos_profile_sensor_data = None
    sensor_msgs_msg.NavSatFix = type("NavSatFix", (), {})
    import gnss_relay

    dpm, profile = gnss_relay._load_profile(str(PROFILES_FILE), "noisy")
    assert dpm == 1.0e-5
    assert profile["bias_east_m"] == 1.2
    with pytest.raises(KeyError):
        gnss_relay._load_profile(str(PROFILES_FILE), "does-not-exist")

    # Degradation math: the noisy profile moves the fix by its constant
    # bias plus seeded noise, deterministically for a given seed.
    numpy = __import__("numpy")
    rng_a = numpy.random.RandomState(7)
    rng_b = numpy.random.RandomState(7)
    fix_a = gnss_relay.degrade_fix(
        40.0, -73.0, 5.0, rng=rng_a, sigma_h_m=5.0, sigma_v_m=10.0,
        bias_east_m=1.2, bias_north_m=1.6, degrees_per_metre=dpm,
    )
    fix_b = gnss_relay.degrade_fix(
        40.0, -73.0, 5.0, rng=rng_b, sigma_h_m=5.0, sigma_v_m=10.0,
        bias_east_m=1.2, bias_north_m=1.6, degrees_per_metre=dpm,
    )
    assert fix_a == fix_b  # same seed replays the same degradation
    assert fix_a != (40.0, -73.0, 5.0)  # the profile actually moved the fix
    lat, lon, alt = fix_a
    # bias 1.6 m + noise sigma 5 m per axis at 1e-5 deg/m: |offset| < 30 m
    # is beyond five sigmas of the per-axis model.
    assert abs(lat - 40.0) < 3e-4
    assert abs(lon + 73.0) < 3e-4
    assert abs(alt - 5.0) < 60.0  # 10 m sigma, no bias

    # Dropout windows: inside a window the fix is dropped entirely.
    assert gnss_relay.is_dropped(16.0, [(15.0, 17.0)])
    assert not gnss_relay.is_dropped(14.99, [(15.0, 17.0)])
    assert not gnss_relay.is_dropped(17.0, [(15.0, 17.0)])


# ------------------------------------------------------------ SR pin


def test_scenario_runner_pin_is_wellformed():
    pin = yaml.safe_load(PIN_FILE.read_text())
    assert pin["tag"] == f"v{pin['version']}"
    assert re.fullmatch(r"[0-9a-f]{40}", pin["commit"])
    assert pin["repository"].endswith("scenario_runner.git")
    assert pin["path"]


# ------------------------------------------------------------ .xosc files


def test_four_locked_scenarios_exist():
    names = {p.stem for p in SCENARIOS}
    assert {"S1_nominal", "S2_noisy_gnss", "S3_turning_high_speed",
            "S4_intersection_cut_in"} <= names


@pytest.mark.parametrize("scenario_file", SCENARIOS, ids=lambda p: p.stem)
def test_scenario_contract(scenario_file):
    root = ET.parse(scenario_file).getroot()
    header = root.find("FileHeader")
    assert header.attrib["description"].startswith("CARLA:")
    assert root.find("RoadNetwork/LogicFile").attrib["filepath"] == "Town05"

    # The labelled open-loop harness from #10 drives the ego.
    modules = [p.attrib.get("value") for p in root.iter("Property")
               if p.attrib.get("name") == "module"]
    assert "carla_ros_scenario_runner.ros_vehicle_control" in modules

    # Every scenario ends on explicit criteria (#10 §2).
    criteria = [c.attrib["name"] for c in root.iter("Condition")
                if c.attrib.get("name", "").startswith("criteria_")]
    assert "criteria_CollisionTest" in criteria

    # Route stays inside the map by construction: waypoints are literals.
    waypoints = list(root.iter("Waypoint"))
    assert len(waypoints) >= 2

    # Fixed clear/dry weather in Init.
    assert root.find("Storyboard/Init") is not None
    assert root.find(".//EnvironmentAction") is not None


def test_s4_has_signalized_cut_in():
    root = ET.parse(REPO / "scenarios/S4_intersection_cut_in.xosc").getroot()
    criteria = [c.attrib["name"] for c in root.iter("Condition")
                if c.attrib.get("name", "").startswith("criteria_")]
    assert "criteria_RunningRedLightTest" in criteria
    signals = [s.attrib for s in root.iter("TrafficSignalStateAction")]
    states = {s["state"] for s in signals}
    assert {"red", "green"} <= states
    # The cut-in adversary merges into the ego lane.
    lane_changes = list(root.iter("LaneChangeAction"))
    assert len(lane_changes) == 1
    assert lane_changes[0].find("LaneChangeTarget/RelativeTargetLane").attrib["value"] == "-1"
    assert list(root.iter("AddEntityAction")), "adversary must be spawned by the story"


def test_s2_and_s1_share_the_route():
    s1 = [w for w in ET.parse(REPO / "scenarios/S1_nominal.xosc").getroot().iter("Waypoint")]
    s2 = [w for w in ET.parse(REPO / "scenarios/S2_noisy_gnss.xosc").getroot().iter("Waypoint")]
    assert len(s1) == len(s2)
    for a, b in zip(s1, s2):
        assert a.find("Position/WorldPosition").attrib == \
            b.find("Position/WorldPosition").attrib


# ------------------------------------------------------------ coverage


def test_coordinate_chain_y_flips_cancel():
    """OSC (x, y) must land at ROS-world (x, y): SR flips y once when
    reading the OSC, the bridge flips y once converting CARLA->ROS."""
    # Pure translation for easy hand-checking.
    t = [[1, 0, 0, 100], [0, 1, 0, -50], [0, 0, 1, 0], [0, 0, 0, 1]]
    mapped = check_route_coverage.osc_to_map([(10.0, -20.0, 0.0)], t)
    assert mapped[0][0] == pytest.approx(110.0)
    assert mapped[0][1] == pytest.approx(-70.0)


def test_route_outside_coverage_fails(tmp_path):
    trajectory = tmp_path / "trajectory.tum"
    trajectory.write_text(
        "0.0 0.0 0.0 0 0 0 1\n0.0 0.0 1.0 0 0 0 1\n0.0 0.0 2.0 0 0 0 1\n"
    )
    scenario = tmp_path / "off_route.xosc"
    scenario.write_text(textwrap.dedent("""\
        <OpenSCENARIO>
          <Storyboard><Story><Act>
            <ManeuverGroup><Maneuver><Event><Action>
              <PrivateAction><RoutingAction><AssignRouteAction>
                <Route name="r">
                  <Waypoint><Position><WorldPosition x="0" y="0" z="0"/></Position></Waypoint>
                  <Waypoint><Position><WorldPosition x="90" y="0" z="0"/></Position></Waypoint>
                </Route>
              </AssignRouteAction></RoutingAction></PrivateAction>
            </Action></Event></Maneuver></ManeuverGroup>
          </Act></Story></Storyboard>
        </OpenSCENARIO>
    """))
    summary = check_route_coverage.run_preflight(
        str(scenario), _write_map_eval(tmp_path), str(trajectory)
    )
    assert not summary["inside_coverage"]
    assert summary["max_distance_m"] > 20.0


def test_route_inside_coverage_passes(tmp_path):
    trajectory = tmp_path / "trajectory.tum"
    # TUM rows are "timestamp tx ty tz ..."; this trajectory runs down -y.
    trajectory.write_text(
        "0.0 0.0 0.0 0 0 0 1\n0.0 0.0 -5.0 0 0 0 1\n0.0 0.0 -10.0 0 0 0 1\n"
    )
    scenario = tmp_path / "on_route.xosc"
    scenario.write_text(textwrap.dedent("""\
        <OpenSCENARIO>
          <Storyboard><Story><Act>
            <ManeuverGroup><Maneuver><Event><Action>
              <PrivateAction><RoutingAction><AssignRouteAction>
                <Route name="r">
                  <Waypoint><Position><WorldPosition x="0.5" y="0.0" z="0"/></Position></Waypoint>
                  <Waypoint><Position><WorldPosition x="0.5" y="-9.5" z="0"/></Position></Waypoint>
                </Route>
              </AssignRouteAction></RoutingAction></PrivateAction>
            </Action></Event></Maneuver></ManeuverGroup>
          </Act></Story></Storyboard>
        </OpenSCENARIO>
    """))
    summary = check_route_coverage.run_preflight(
        str(scenario), _write_map_eval(tmp_path), str(trajectory)
    )
    assert summary["inside_coverage"]
    assert summary["max_distance_m"] < 1.0


def _write_map_eval(tmp_path):
    import json
    map_eval = tmp_path / "map_eval.yaml"
    map_eval.write_text(json.dumps({
        "transform_semantics": "ros_base_link_to_map_base_link",
        "T_map_carla": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    }))
    return str(map_eval)
