import pytest

from ament_flake8.main import main

def test_flake8():
    rc = main(argv=['.'])
    assert rc == 0, 'Found errors'
