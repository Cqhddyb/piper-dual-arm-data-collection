#!/usr/bin/env python3
"""Entry point; source checkout works without an editable installation."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from piper_collection.diagnostics.dataset import main

if __name__ == "__main__":
    main()
