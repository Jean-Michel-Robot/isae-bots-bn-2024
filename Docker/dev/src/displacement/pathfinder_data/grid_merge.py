#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#     ____                                                  
#    / ___| _   _ _ __   __ _  ___ _ __ ___                 
#    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
#     ___) | |_| | |_) | (_| |  __/ | | (_) |               
#    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
#   ____       _ |_|       _   _ _       ____ _       _     
#  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
#  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
#  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
#  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 

# pyright: reportMissingImports=false

import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

ifile1 = "./match_grid_home.xml"
ifile2 = "./match_grid_away.xml"
ofile = "./match_grid_complete.xml"

idata1 = ET.parse(os.path.join(os.path.dirname(__file__), ifile1))
idata2 = ET.parse(os.path.join(os.path.dirname(__file__), ifile2))
odata = ET.Element('map')

offset = len(idata1.xpath("/map/node"))
for nodeParam in idata1.xpath("/map/node"):
    node = ET.SubElement(odata, "node")
    node.set("id","p{}".format(int(nodeParam.get('id')[1:])))
    node.set("x", "{}".format(int(nodeParam.get("x"))))
    node.set("y", "{}".format(int(nodeParam.get("y"))))
for nodeParam in idata2.xpath("/map/node"):
    node = ET.SubElement(odata, "node")
    node.set("id","p{}".format(int(nodeParam.get('id')[1:]) + offset))
    node.set("x", "{}".format(int(nodeParam.get("x"))))
    node.set("y", "{}".format(int(nodeParam.get("y"))))

for linkParam in idata1.xpath("/map/connection"):
    link = ET.SubElement(odata, "connection")
    link.set("p1","p{}".format(int(linkParam.get("p1")[1:])))
    link.set("p2","p{}".format(int(linkParam.get("p2")[1:])))
for linkParam in idata2.xpath("/map/connection"):
    link = ET.SubElement(odata, "connection")
    link.set("p1","p{}".format(int(linkParam.get("p1")[1:]) + offset))
    link.set("p2","p{}".format(int(linkParam.get("p2")[1:]) + offset))
    
fid = open(ofile, "w")
xmlstr = minidom.parseString(ET.tostring(odata)).toprettyxml(indent="   ")
fid.write(xmlstr)
fid.close()

