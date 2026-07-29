from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_active_adapter_uses_only_the_canonical_social_message():
    source = (
        PACKAGE_ROOT / 'semaforr_crowd' / 'node.py'
    ).read_text(encoding='utf-8')

    assert 'SocialObservation' in source
    assert 'prediction_stamps' in source
    assert 'semaforr.msg import CrowdModel' not in source
    assert 'crowd_pose' not in source
    assert 'rospy' not in source


def test_ros2_package_installs_a_validated_default_configuration():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    config = PACKAGE_ROOT / 'config' / 'crowd.yaml'

    assert 'crowd_model = semaforr_crowd.node:main' in setup
    assert config.is_file()
    assert 'social_observations' in config.read_text(encoding='utf-8')


def test_diagnostic_launch_connects_the_migrated_packages():
    launch = (
        PACKAGE_ROOT / 'launch' / 'social_diagnostics.launch.py'
    ).read_text(encoding='utf-8')

    assert "package='semaforr_crowd'" in launch
    assert "package='why'" in launch
    assert "package='why_plan'" in launch
