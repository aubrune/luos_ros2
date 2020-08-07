#!/usr/bin/env python3

from luos_interface.modules import _make as modules
from luos_interface.modules import *

for name, Class in modules.items():
    # Call the publisher's constructor with node = None to print the doc

    print("# Module", name)
    print("## ROS topics")
    print("| **Topic name** | **Message type** |")
    print("|:----|:---:|")

    c = Class(None, None, None)
    print(c.get_markdown_doc())
    print("\n\n")