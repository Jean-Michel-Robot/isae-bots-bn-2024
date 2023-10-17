#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pyright: reportMissingImports=false

"""Generates a grid for a map of the table."""

from lxml import etree

ofile = "./WholeTableGrid_away.xml"

margin = 100                            # entre les noeuds des bords et les bords
min_x, max_x = 0+margin, 2000-margin    # Dimensions en x
min_y, max_y = 1500+margin, 3000-margin # Dimensions en y

dx, dy = max_x-min_x, max_y-min_y 
nbPts_h = 20                            # nb de points selon x
nbPts_w = nbPts_h * dy // dx            # nb de points selon y
x_step = (max_x-min_x) / (nbPts_h-1)    # ecart entre les points en x
y_step = (max_y-min_y) / (nbPts_w-1)    # ecart entre les points en y

# Generation
grid = etree.Element("map")
counter = 0
for xId in range(nbPts_h):
    for yId in range(nbPts_w):
        node = etree.SubElement(grid, "node")
        node.set("id","p{}".format(counter))
        node.set("x","{}".format(min_x + int(xId * x_step)))
        node.set("y","{}".format(min_y + int(yId * y_step)))

        if(yId != nbPts_w-1):
            link = etree.SubElement(grid, "connection")
            link.set("p1","p{}".format(counter))
            link.set("p2","p{}".format(counter+1))            
            if(xId != nbPts_h-1):
                connectionB = etree.SubElement(grid, "connection")
                connectionB.set("p1","p{}".format(counter))
                connectionB.set("p2","p{}".format(counter+nbPts_w))
        else:
            if(xId != nbPts_h-1):
                connectionB = etree.SubElement(grid, "connection")
                connectionB.set("p1","p{}".format(counter))
                connectionB.set("p2","p{}".format(counter+nbPts_w))
        counter+=1

fid = open(ofile, "w")
fid.write(etree.tostring(grid, encoding='utf-8', xml_declaration=True, pretty_print=True))
fid.close()
