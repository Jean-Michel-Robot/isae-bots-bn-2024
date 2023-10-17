#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""
Take a node_grid as input and return a reversed grid as output.
Grids must be passed as *.xml files.
Reverse means going from 'home' to 'away' and vice & versa.
"""

import os
from lxml import etree

ifile = "./match_grid_home.xml"
ofile = "./match_grid_away.xml"

idata = etree.parse(os.path.join(os.path.dirname(__file__), ifile))
odata = etree.Element('map')

for nodeParam in idata.xpath("/map/node"):
    node = etree.SubElement(odata, "node")
    node.set("id","p{}".format(int(nodeParam.get('id')[1:])))
    node.set("x", "{}".format(int(nodeParam.get("x"))))
    node.set("y", "{}".format(3000-int(nodeParam.get("y"))))
for linkParam in idata.xpath("/map/connection"):
    link = etree.SubElement(odata, "connection")
    link.set("p1","p{}".format(int(linkParam.get("p1")[1:])))
    link.set("p2","p{}".format(int(linkParam.get("p2")[1:])))

fid = open(ofile, "w")
fid.write(etree.tostring(odata, encoding='utf-8', xml_declaration=True, pretty_print=True))
fid.close()

