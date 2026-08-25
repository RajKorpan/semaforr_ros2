from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_package_contains_no_crowd_model_or_ros_transport():
    source = (PACKAGE_ROOT / 'semaforr_crowd' / 'node.py').read_text(
        encoding='utf-8'
    )
    assert 'CrowdField' not in source
    assert not (PACKAGE_ROOT / 'semaforr_crowd' / 'model.py').exists()
    assert not (PACKAGE_ROOT / 'config' / 'crowd.yaml').exists()


def test_compatibility_executable_points_users_to_semaforr():
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    source = (PACKAGE_ROOT / 'semaforr_crowd' / 'node.py').read_text(
        encoding='utf-8'
    )
    assert 'crowd_model = semaforr_crowd.node:main' in setup
    assert 'SemaFORR owns crowd' in source
