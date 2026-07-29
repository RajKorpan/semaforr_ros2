from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_active_adapter_consumes_only_the_derived_crowd_field():
    source = (
        PACKAGE_ROOT / 'semaforr_crowd' / 'node.py'
    ).read_text(encoding='utf-8')

    assert 'CrowdField' in source
    assert 'SocialObservation' not in source
    assert 'semaforr.msg import CrowdModel' not in source
    assert 'crowd_pose' not in source
    assert 'rospy' not in source


def test_ros2_package_installs_a_validated_default_configuration():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    config = PACKAGE_ROOT / 'config' / 'crowd.yaml'

    assert 'crowd_model = semaforr_crowd.node:main' in setup
    assert config.is_file()
    assert 'crowd_field' in config.read_text(encoding='utf-8')


def test_ros1_learner_variants_are_explicitly_quarantined():
    legacy_variants = (
        'crowd_bayes_cusum',
        'crowd_behavior',
        'crowd_count',
        'crowd_count_thompson',
        'crowd_cusum',
        'crowd_learner',
    )
    assert all(
        (PACKAGE_ROOT / variant / 'COLCON_IGNORE').is_file()
        for variant in legacy_variants
    )


def test_diagnostic_launch_connects_the_migrated_packages():
    launch = (
        PACKAGE_ROOT / 'launch' / 'social_diagnostics.launch.py'
    ).read_text(encoding='utf-8')

    assert "package='semaforr_crowd'" in launch
    assert "package='why'" in launch
    assert "package='why_plan'" in launch
