"""
ConfigReader.py

This module provides functionality to read and validate a YAML configuration file. 
It ensures that all required fields are present and assigns default values to optional fields if not provided.

Classes:
    - ConfigReader: Handles loading and validating the YAML configuration file.

Usage:
    from ConfigReader import ConfigReader

    config_reader = ConfigReader("path/to/config.yml")
    config = config_reader.load_config()
    print(config)

Example:
    $ python load_config.py path/to/config.yml
"""
__author__ = "shanakaprageeth"

import yaml
import argparse
import os
import logging


class ConfigReader:
    # pylint: disable=too-few-public-methods
    def __init__(self, config_path):
        # Convert relative path to absolute path
        self.config_path = os.path.abspath(config_path)
        self.config = None

    def load_config(self):
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        if not self.config_path.endswith('.yml') and not self.config_path.endswith('.yaml'):
            raise ValueError("Config file must be in YAML format (.yml or .yaml)")
        
        try:
            with open(self.config_path, 'r') as file:
                self.config = yaml.safe_load(file)
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file: {e}")
        
        self.validate_config()
        return self.config

    def validate_config(self):
        # assign default values to optional fields
        required_fields = {
            "mqtthost": None, 
            "mqttport": None, 
            "spfx": None, 
            "tomdop": None, 
            "tomcop": None, 
            "tomd0op": None, 
            "loc": None, 
            "locclient": None, 
            "TEMPTEDS": None, 
            "HUMIDTEDS": None, 
            "SERVOTEDS": None, 
            "SECURITYTEDS": None, 
            "TEMPBINTEDS": None, 
            "HUMIDBINTEDS": None, 
            "SERVOBINTEDS": None, 
            "SECURITYBINTEDS": None
        }

        for field, default_value in required_fields.items():
            if field not in self.config:
                if default_value is not None:
                    logging.warning(f"Field '{field}' is not provided. Using default value: {default_value}")
                    self.config[field] = default_value
                else:
                    raise ValueError(f"Missing required field: {field}")
        
        if not isinstance(self.config.get("mqttport"), int):
            raise ValueError("Field 'mqttport' must be an integer.")
        
        # Check optional fields
        if "username" not in self.config:
            logging.warning("Optional field 'username' is not provided. Setting it to an empty value.")
            self.config["username"] = ""
        if "password" not in self.config:
            logging.warning("Optional field 'password' is not provided. Setting it to an empty value.")
            self.config["password"] = ""

