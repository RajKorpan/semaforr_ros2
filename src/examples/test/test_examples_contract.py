import os
from pathlib import Path


PACKAGE_ROOT = Path(os.environ['SEMAFORR_EXAMPLES_SOURCE_DIR'])


def test_legacy_scenario_corpus_is_preserved_as_data():
    core = PACKAGE_ROOT / 'core'
    files = [path for path in core.rglob('*') if path.is_file()]

    assert len(files) >= 500
    assert any(path.suffix == '.xml' for path in files)
    assert any(path.suffix == '.conf' for path in files)


def test_package_installs_assets_beneath_its_share_directory():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert 'DIRECTORY core' in cmake
    assert 'DESTINATION share/${PROJECT_NAME}' in cmake
