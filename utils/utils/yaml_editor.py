import yaml


class YamlEditor:
    def __init__(self):
        pass

    def read(self, file_path):
        with open(file_path, "r") as yaml_file:
            data = yaml.safe_load(yaml_file)
        return data

    def save(self, file_path, data_dict, key=None):
        with open(file_path, 'r') as yaml_file:
            # Reading the existing data from the YAML file
            yaml_object = yaml.safe_load(yaml_file) or {}

        if key is not None:
            yaml_object[key] = data_dict
        else:
            yaml_object = data_dict

        with open(file_path, 'w') as yaml_file:
            yaml.safe_dump(yaml_object, yaml_file, default_flow_style=False)
