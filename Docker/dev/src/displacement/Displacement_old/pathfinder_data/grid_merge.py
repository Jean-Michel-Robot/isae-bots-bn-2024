#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

import os
from lxml import etree

ifile1 = "./match_grid_home.xml"
ifile2 = "./match_grid_away.xml"
ofile = "./match_grid_complete.xml"

idata1 = etree.parse(os.path.join(os.path.dirname(__file__), ifile1))
idata2 = etree.parse(os.path.join(os.path.dirname(__file__), ifile2))
odata = etree.Element('map')

offset = len(idata1.xpath("/map/node"))
for nodeParam in idata1.xpath("/map/node"):
    node = etree.SubElement(odata, "node")
    node.set("id","p{}".format(int(nodeParam.get('id')[1:])))
    node.set("x", "{}".format(int(nodeParam.get("x"))))
    node.set("y", "{}".format(int(nodeParam.get("y"))))
for nodeParam in idata2.xpath("/map/node"):
    node = etree.SubElement(odata, "node")
    node.set("id","p{}".format(int(nodeParam.get('id')[1:]) + offset))
    node.set("x", "{}".format(int(nodeParam.get("x"))))
    node.set("y", "{}".format(int(nodeParam.get("y"))))

for linkParam in idata1.xpath("/map/connection"):
    link = etree.SubElement(odata, "connection")
    link.set("p1","p{}".format(int(linkParam.get("p1")[1:])))
    link.set("p2","p{}".format(int(linkParam.get("p2")[1:])))
for linkParam in idata2.xpath("/map/connection"):
    link = etree.SubElement(odata, "connection")
    link.set("p1","p{}".format(int(linkParam.get("p1")[1:]) + offset))
    link.set("p2","p{}".format(int(linkParam.get("p2")[1:]) + offset))
    
fid = open(ofile, "w")
fid.write(etree.tostring(odata, encoding='utf-8', xml_declaration=True, pretty_print=True))
fid.close()

