import unittest
import os
import numpy as np
import yaml
from orca_core.utils import update_yaml, read_yaml

class TestYamlFunctions(unittest.TestCase):

    def setUp(self):
        self.test_file = 'test.yaml'
        self.initial_data = {
            'key1': 'value1',
            'key2': [1, 2, 3],
            'key3': {'subkey1': 'subvalue1'}
        }
        with open(self.test_file, 'w') as file:
            yaml.dump(self.initial_data, file, default_flow_style=False, sort_keys=False)

    def tearDown(self):
        if os.path.exists(self.test_file):
            os.remove(self.test_file)

    def test_update_yaml_with_string(self):
        update_yaml(self.test_file, 'key1', 'new_value1')
        data = read_yaml(self.test_file)
        self.assertEqual(data['key1'], 'new_value1')

    def test_update_yaml_with_list(self):
        update_yaml(self.test_file, 'key2', [4, 5, 6])
        data = read_yaml(self.test_file)
        self.assertEqual(data['key2'], [4, 5, 6])

    def test_update_yaml_with_dict(self):
        update_yaml(self.test_file, 'key3', {'subkey2': 'subvalue2'})
        data = read_yaml(self.test_file)
        self.assertEqual(data['key3'], {'subkey2': 'subvalue2'})

    def test_update_yaml_with_numpy_array(self):
        array = np.array([7, 8, 9])
        update_yaml(self.test_file, 'key4', array)
        data = read_yaml(self.test_file)
        self.assertEqual(data['key4'], [7, 8, 9])

    def test_read_yaml(self):
        data = read_yaml(self.test_file)
        self.assertEqual(data, self.initial_data)

    def test_update_yaml_creates_file(self):
        new_file = 'new_test.yaml'
        try:
            update_yaml(new_file, 'key1', 'value1')
            data = read_yaml(new_file)
            self.assertEqual(data['key1'], 'value1')
        finally:
            if os.path.exists(new_file):
                os.remove(new_file)

if __name__ == '__main__':
    unittest.main()