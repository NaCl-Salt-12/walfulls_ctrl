import subprocess
import os
import sys
import re
from pathlib import Path


exper_name = os.getenv("EXPERIMENT_NAME", "default_experiment")


def find_dir(start: Path, substr: str):
    return (p for p in start.rglob("*") if p.is_dir() and substr in p.name)
