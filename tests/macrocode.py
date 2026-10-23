"""Load a macro's pure-Python part (its constants and its filter / geometry functions) without TBC.

Each macro is laid out as: imports (TBC / .NET), constants, Setup(), a section of pure-Python
functions starting with a "# ---...--- <name> (pure Python)" comment, then
"# ---...--- TBC side". The constants and that section are executed in a plain namespace, so tests
run exactly the code the macro runs in TBC."""
import math
import os
import random
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MACROS = os.path.join(HERE, "..", "PointCloudTools")
SECTION = "# ----------------------------------------------------------------"


def load(name_or_path):
    """Namespace with the macro's constants and pure-Python functions. Accepts a macro name
    ("CleanCloud") or a path to a .py file."""
    path = name_or_path if name_or_path.endswith(".py") else os.path.join(MACROS, name_or_path + ".py")
    src = open(path, encoding="utf-8").read()
    constants = src[re.search(r"^[A-Z][A-Z0-9_]* = ", src, re.M).start():src.index("def Setup(")]
    start = src.index(SECTION)
    end = src.index(SECTION + " TBC side")
    ns = {"math": math, "random": random, "time": time}  # the standard modules the macros import
    exec(constants + src[start:end], ns)
    return ns
