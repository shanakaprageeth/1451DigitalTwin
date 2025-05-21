
__author__ = "shanakaprageeth"

import yaml
import argparse
import os
import logging
from py_lib_digitaltwin.ConfigReader import ConfigReader

def config_reader_main():
    # pylint: disable=missing-function-docstring
    parser = argparse.ArgumentParser(description="Read a YAML configuration file.")
    parser.add_argument('config_path', type=str, help="Path to the YAML configuration file (relative or absolute).")
    args = parser.parse_args()

    try:
        config_reader = ConfigReader(args.config_path)
        config = config_reader.load_config()
        print("Configuration loaded successfully:")
        print(config)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # pylint: disable=invalid-name
    config_reader_main()


