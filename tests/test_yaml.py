import os

import numpy as np
import pytest
import yaml
from orca_core.utils import update_yaml, read_yaml


@pytest.fixture
def yaml_file(tmp_path):
    test_file = tmp_path / "test.yaml"
    initial_data = {
        'key1': 'value1',
        'key2': [1, 2, 3],
        'key3': {'subkey1': 'subvalue1'},
    }
    with open(test_file, 'w') as f:
        yaml.dump(initial_data, f, default_flow_style=False, sort_keys=False)
    return str(test_file), initial_data


def test_update_yaml_with_string(yaml_file):
    test_file, _ = yaml_file
    update_yaml(test_file, 'key1', 'new_value1')
    data = read_yaml(test_file)
    assert data['key1'] == 'new_value1'


def test_update_yaml_with_list(yaml_file):
    test_file, _ = yaml_file
    update_yaml(test_file, 'key2', [4, 5, 6])
    data = read_yaml(test_file)
    assert data['key2'] == [4, 5, 6]


def test_update_yaml_with_dict(yaml_file):
    test_file, _ = yaml_file
    update_yaml(test_file, 'key3', {'subkey2': 'subvalue2'})
    data = read_yaml(test_file)
    assert data['key3'] == {'subkey2': 'subvalue2'}


def test_update_yaml_with_numpy_array(yaml_file):
    test_file, _ = yaml_file
    array = np.array([7, 8, 9])
    update_yaml(test_file, 'key4', array)
    data = read_yaml(test_file)
    assert data['key4'] == [7, 8, 9]


def test_read_yaml(yaml_file):
    test_file, initial_data = yaml_file
    data = read_yaml(test_file)
    assert data == initial_data


def test_update_yaml_creates_file(tmp_path):
    new_file = str(tmp_path / "new_test.yaml")
    update_yaml(new_file, 'key1', 'value1')
    data = read_yaml(new_file)
    assert data['key1'] == 'value1'
