#!/usr/bin/env python3
"""Validate ROS package layout and ownership of runtime settings without starting ROS."""
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


class UniqueMappingLoader(yaml.SafeLoader):
    def construct_mapping(self, node, deep=False):
        keys = [self.construct_object(key) for key, _ in node.value]
        if len(set(keys)) != len(keys):
            raise ValueError(f"duplicate YAML key near line {node.start_mark.line + 1}")
        return super().construct_mapping(node, deep)


def inspect(workspace):
    source = workspace / "src"
    packages = {}
    owners = {}
    count = 0
    for folder, dirs, files in os.walk(source):
        path = Path(folder)
        bad = set(dirs) & {"build", "install", "log", ".git"}
        if bad:
            raise ValueError(f"{path}: unexpected generated or nested workspace directories {bad}")
        if "package.xml" not in files:
            continue
        name = ET.parse(path / "package.xml").findtext("name")
        if name != path.name or name in packages:
            raise ValueError(f"{path}: package name {name!r} is duplicated or differs from directory")
        packages[name] = path
        config_dir = path / "config"
        if not config_dir.is_dir():
            continue
        owners[config_dir] = {}
        for file in sorted(config_dir.glob("*.yaml")):
            try:
                document = yaml.load(file.read_text(), Loader=UniqueMappingLoader)
                # A default category belongs to the one node that loads it.
                if len(document) != 1:
                    raise ValueError("expected a single ROS node key")
                values = next(iter(document.values()))["ros__parameters"]
                if not isinstance(values, dict) or not values:
                    raise ValueError("expected nonempty ros__parameters")
                for key in values:
                    previous = owners[config_dir].get(key)
                    if previous is not None:
                        raise ValueError(f"{key!r} already belongs to {previous.name}")
                    owners[config_dir][key] = file
            except (ValueError, TypeError, KeyError, yaml.YAMLError) as error:
                raise ValueError(f"{file}: {error}") from error
            count += 1
        if not owners[config_dir]:
            raise ValueError(f"{config_dir}: no runtime defaults found")
    links = [p for p in (workspace / "config").iterdir() if p.is_symlink()]
    targets = {p.resolve() for p in links}
    if len(links) != len(targets) or targets != set(owners):
        raise ValueError(f"config index does not match the package config directories: {targets ^ set(owners)}")
    if not packages or not count:
        raise ValueError("no packages or config files found")
    print(f"{len(packages)} ROS packages; {count} category files; "
          f"{sum(len(entries) for entries in owners.values())} settings.")


if __name__ == "__main__":
    try:
        inspect(Path(__file__).resolve().parents[1])
    except (OSError, ValueError, ET.ParseError) as error:
        sys.exit(str(error))
