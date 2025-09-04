import pytest

from ament_copyright.main import main

def test_copyright():
    rc = main(argv=['.'])
    assert rc == 0, 'Found errors'
