"""Static checks of the expanded SO-101 URDF."""

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro

PKG = os.path.join(os.path.dirname(__file__), '..')
XACRO = os.path.join(PKG, 'urdf', 'so101.urdf.xacro')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def expand(**mappings):
    return xacro.process_file(XACRO, mappings=mappings).toxml()


@pytest.fixture(params=['true', 'false'], ids=['mock', 'real'])
def urdf(request):
    return ET.fromstring(
        expand(use_mock_hardware=request.param, calibration_file='/tmp/cal.json'))


def test_has_six_actuated_joints(urdf):
    revolute = [j.get('name') for j in urdf.findall('joint') if j.get('type') == 'revolute']
    assert sorted(revolute) == sorted(JOINTS)


def test_ros2_control_limits_match_urdf_limits(urdf):
    """The plugin clamps/maps using the command interface min/max, so they
    must never drift from the kinematic limits."""
    limits = {
        j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
        for j in urdf.findall('joint') if j.get('type') == 'revolute'
    }
    ros2_control = urdf.find('ros2_control')
    for joint in ros2_control.findall('joint'):
        cmd = joint.find("command_interface[@name='position']")
        params = {p.get('name'): float(p.text) for p in cmd.findall('param')}
        assert (params['min'], params['max']) == pytest.approx(limits[joint.get('name')])


def test_meshes_exist(urdf):
    for mesh in urdf.iter('mesh'):
        filename = mesh.get('filename')
        prefix = 'package://so101_description/'
        assert filename.startswith(prefix), filename
        assert os.path.isfile(os.path.join(PKG, filename[len(prefix):])), filename


def test_real_hardware_uses_so101_plugin():
    urdf = ET.fromstring(expand(use_mock_hardware='false', calibration_file='/tmp/cal.json'))
    hw = urdf.find('ros2_control/hardware')
    assert hw.find('plugin').text == 'so101_hardware/SO101SystemHardware'
    params = {p.get('name'): p.text for p in hw.findall('param')}
    assert params['calibration_file'] == '/tmp/cal.json'
    gripper = urdf.find("ros2_control/joint[@name='gripper']")
    assert gripper.find("param[@name='mapping']").text == 'linear'


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='urdfdom tools not installed')
def test_check_urdf(tmp_path):
    out = tmp_path / 'so101.urdf'
    out.write_text(expand())
    result = subprocess.run(['check_urdf', str(out)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
