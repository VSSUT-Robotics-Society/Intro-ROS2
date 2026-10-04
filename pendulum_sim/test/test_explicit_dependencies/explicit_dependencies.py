"""Audit for undeclared dependencies in an ament_python ROS package."""

from pathlib import Path

from python_dependencies import get_py_imports
from xml_dependencies import (
    get_package_xml_dependencies,
    get_xacro_dependencies,
)

IGNORED_DIRS: set[str] = {'__pycache__', '.pytest_cache', 'test'}


def get_package_root() -> Path:
    """Return the root directory of the package under test."""
    return Path.cwd()


def get_declared_dependencies(package_root: Path) -> set[str]:
    """Return dependencies declared in package.xml."""
    return get_package_xml_dependencies(package_root / 'package.xml')


def get_python_dependencies(package_root: Path) -> set[str]:
    """Discover external dependencies imported by Python source files."""
    found: set[str] = set()

    for file_path in package_root.rglob('*.py'):
        if any(part in file_path.parts for part in IGNORED_DIRS):
            continue
        if file_path.name == 'setup.py':
            continue
        found.update(get_py_imports(file_path))

    found.discard(package_root.name)
    return found


def get_xml_dependencies(package_root: Path) -> set[str]:
    """Discover dependencies referenced by XML, URDF, and Xacro files."""
    found: set[str] = set()

    for file_path in package_root.rglob('*'):
        if any(part in file_path.parts for part in IGNORED_DIRS):
            continue
        if file_path.suffix.lower() in {'.xacro', '.urdf', '.xml'}:
            found.update(get_xacro_dependencies(file_path))

    found.discard(package_root.name)
    return found


def assert_declared_dependencies(
        kind: str, found: set[str], declared: set[str]) -> None:
    """Assert that all discovered dependencies are declared in package.xml."""
    missing = found - declared
    assert not missing, (
        f'Missing {kind} dependencies in package.xml: {sorted(missing)}\n'
        f'Found: {sorted(found)}\n'
        f'Declared: {sorted(declared)}'
    )


def test_python_dependencies():
    """Check that Python dependencies are declared in package.xml."""
    package_root = get_package_root()
    found = get_python_dependencies(package_root)
    declared = get_declared_dependencies(package_root)
    assert_declared_dependencies('Python', found, declared)


def test_xml_dependencies():
    """Check that XML dependencies are declared in package.xml."""
    package_root = get_package_root()
    found = get_xml_dependencies(package_root)
    declared = get_declared_dependencies(package_root)
    assert_declared_dependencies('XML', found, declared)
