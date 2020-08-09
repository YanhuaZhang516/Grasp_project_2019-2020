from mujoco_py import load_model_from_xml
import os
import xml.dom.minidom
import xml.etree.ElementTree as ET
import io
import numpy as np

from robosuite.utils import XMLError

fname=""

tree = ET.parse(fname)
root = self.tree.getroot()
with io.StringIO() as string:
    string.write(ET.tostring(self.root, encoding="unicode"))

    model = load_model_from_xml(string.getvalue())