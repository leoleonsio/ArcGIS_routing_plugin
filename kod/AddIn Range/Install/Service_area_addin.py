import arcpy
import pythonaddins
import os
import math
from math import sqrt, degrees, atan2
import heapq
import datetime
import numpy as np


class calcservice(object):
    """Implementation for Service_area_addin.calculate_button (Button)"""
    def __init__(self):
        self.enabled = False
        self.checked = False
        
    def onClick(self):
        arcpy.env.workspace = os.path.abspath('./')
        arcpy.env.overwriteOutput = True
        inFeatures = layer_combobox.value
        unit = unit_ombobox.value
        range = int(range_combobox.value)
        cursor = arcpy.da.UpdateCursor('./point.shp', ["SHAPE@XY"])
        for row in cursor:
            xstart = row[0][0]
            ystart = row[0][1]
            
        startNodeId = 'X' + format(xstart, '.1f') + 'Y' + format(ystart, '.1f')

        newGraph = Graph(inFeatures, unit)
        
        #obtain and visualize limiting edges
        limitingEdges, endNodes = dijkstra(newGraph, startNodeId, range)
        createArea(inFeatures, limitingEdges, endNodes, range, unit)
        del newGraph
        del limitingEdges

class layerselection(object):
    """Implementation for Service_area_addin.layer_combobox (ComboBox)"""
    def __init__(self):
        self.items = ['']
        self.editable = True
        self.enabled = True
        self.dropdownWidth = '123451234512345'
        self.width = '123451234512345'
        self.layerpath = ''
    def onSelChange(self, selection):
        self.value = selection
        desc = arcpy.Describe(selection)
        path = desc.path
        extension = desc.extension
        self.layerpath = str(path) + "/" + selection +'.' + str(extension)
        self.refresh()
        point_tool.enabled = True
        if len(arcpy.ListFields(self.layerpath,'START_X'))>0:
            print('Data already prepared')
        else:
            print('Preparing data...')
            arcpy.AddGeometryAttributes_management(self.layerpath, ["LENGTH","LINE_START_MID_END"])
            print('Data prepared')
    def onFocus(self, focused):
        if focused:  
            self.mxd = arcpy.mapping.MapDocument('current')  
            self.df = self.mxd.activeDataFrame  
            lyrs = [l.name for l in arcpy.mapping.ListLayers(self.mxd, '*', self.df)  
                    if not l.isBroken and l.isFeatureLayer and  
                    arcpy.Describe(l).shapeType == 'Polyline']  
            self.items = sorted(lyrs)

class rangecombobox(object):
    """Implementation for Service_area_addin.range_combobox (ComboBox)"""
    def __init__(self):
        self.items = ['']
        self.editable = True
        self.enabled = False
        self.dropdownWidth = 'WWWWWW'
        self.width = 'WWWWWW'
    def onSelChange(self, selection):
        pass
    def onEditChange(self, text):
        self.value = text
    def onFocus(self, focused):
        pass
    def onEnter(self):
        self.value = int(self.value)
        calculate_button.enabled = True
    def refresh(self):
        pass

class startpoint(object):
    """Implementation for Service_area_addin.point_tool (Tool)"""
    def __init__(self):
        self.enabled = False
        self.first = True
        self.shape = "NONE"  

    def onMouseDown(self, x, y, button, shift):
        pass

    def onMouseDownMap(self, x, y, button, shift):
        # make sure layer exists
        fc = "./point.shp"
        if self.first:
            check(fc, 'point.shp')
            self.first = False

        # check if point exists
        xy = (x, y)
        if int(arcpy.GetCount_management(fc).getOutput(0)) == 0:
            cursor = arcpy.da.InsertCursor(fc, ["SHAPE@XY"])
            cursor.insertRow([xy])
        else:
            cursor = arcpy.da.UpdateCursor(fc, ["SHAPE@XY"])
            for row in cursor:
                row[0] = xy
                cursor.updateRow(row)

        # find nearest road segment to point
        near_features = layer_combobox.layerpath
        search_radius = "100 Meters"
        location = "NO_LOCATION"
        angle = "NO_ANGLE"
        arcpy.Near_analysis(fc, near_features, search_radius, location, angle)
        cursor = arcpy.da.SearchCursor(fc, ["NEAR_FID"])
        for row in cursor:
            if row[0] != -1:
                id = row[0]
            else:
                pythonaddins.MessageBox('Punkt za daleko od drogi', "Error")
                return

        # snap point to nearest segment end
        exp = "FID = " + str(id)
        cursor = arcpy.da.SearchCursor(near_features, ["FID", 'START_X', 'START_Y', 'END_X', 'END_Y'], exp)
        for row in cursor:
            startx = float(row[1])
            starty = float(row[2])
            endx = float(row[3])
            endy = float(row[4])
            l1 = length(x, y, startx, starty)
            l2 = length(x, y, endx, endy)
            if l1 < l2:
                xy = (startx, starty)
            else:
                xy = (endx, endy)
            cursor = arcpy.da.UpdateCursor(fc, ["SHAPE@XY"])
            for row in cursor:
                row[0] = xy
                cursor.updateRow(row)
        # refresh view to see changes
        arcpy.RefreshActiveView()
        arcpy.RefreshTOC()
        unit_ombobox.enabled = True


class unitselection(object):
    """Implementation for Service_area_addin.unit_ombobox (ComboBox)"""
    def __init__(self):
        self.items = ["meters", "minutes"]
        self.editable = True
        self.enabled = False
        self.dropdownWidth = '123456789'
        self.width = '123456789'

    def onSelChange(self, selection):
        range_combobox.enabled = True
        self.value = selection
        self.refresh()
def check(file, name):
    arcpy.env.workspace = os.path.abspath('./')
    if not arcpy.Exists(file):
        arcpy.CreateFeatureclass_management('./', name, "POINT", spatial_reference=2180)
    else:
        mxd = arcpy.mapping.MapDocument('CURRENT')
        df = arcpy.mapping.ListDataFrames(mxd, 'Layers')[0]
        layer = arcpy.mapping.Layer(file)
        arcpy.mapping.AddLayer(df, layer, 'TOP')


def length(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class Node:
    def __init__(self, x, y):
        self.x = round(x, 1)    #small round to correct minimal input data errors
        self.y = round(y, 1)
        self.hash = 'X' + format(self.x, '.1f')+ 'Y' + format(self.y, '.1f')
        self.edges = []
    
    def toString(self):
        return 'Node: \n\t idx: ' + self.hash + '\n\t cords: \n\t\t[' + str(self.x) + ', ' + str(self.y) + ']'
   
class Edge:
    def __init__(self, fid, startNode, endNode, length, speed):
        self.startNode = startNode
        self.endNode = endNode
        self.length = length
        self.speed = speed
        self.hash = startNode.hash + endNode.hash
        self.fid = fid
        
    def toString(self):
        return 'Edge: \n\t nodes: \n\t\t[' + self.startNode.hash + ', ' + self.endNode.hash + '] ' + '\n\t length: ' + str(self.length) + '\n\t cost: ' + str(self.cost)
             
class Graph:
    edges = {}
    nodes = {}
    unit = ''
    endNodes = {} #nodes with no neighbors
    
    def __init__(self, inFeatures, unit = 'meters'):
        self.unit = unit
        costs = {'A':140,'D':30,'G':50,'GP':100,'I':50,'L':50,'S':120,'Z':50}
        repeatingNodes = 0
        repeatingEdges = 0
        cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
            cost = row.length
            metersPerMinute = costs[row.klasaDrogi] * (1000/60)
            if unit == 'minutes':
                cost = cost/metersPerMinute
            startNode = Node(row.start_x, row.start_y)
            endNode = Node(row.end_x, row.end_y)
            newEdge = Edge(row.fid, startNode, endNode, cost, metersPerMinute)
            
            if not self.containsEdge(newEdge):
                self.edges[newEdge.hash] = newEdge
            else:
                repeatingEdges = repeatingEdges + 1
            if not self.containsNode(startNode):   
                self.nodes[startNode.hash] = startNode
            else:
                repeatingNodes = repeatingNodes + 1
            if not self.containsNode(endNode):
                self.nodes[endNode.hash] = endNode
            else:
                repeatingNodes = repeatingNodes + 1
                
            self.nodes[startNode.hash].edges.append(newEdge)
            self.nodes[endNode.hash].edges.append(newEdge)
            
        #print('repeating nodes: ' + str(repeatingNodes) + '\nrepeating edges: ' + str(repeatingEdges))
    def containsNode(self, node):
        if node.hash in self.nodes:
            return 1
        return 0;
    
    def containsEdge(self, edge):
        if edge.hash in self.edges:
            return 1
        return 0;  

    def toString(self):
        return 'Graph: \n\t nodes: ' + str(len(self.nodes)) + ' \n\t edges: ' + str(len(self.edges))
def dijkstra(graph, startNodeId, limit):    
    #returns :dictionary of limiting edges, dictionary of visited nodes without neighbors
    startNode = graph.nodes[startNodeId]
    distances = {node: float('inf') for node in graph.nodes}
    distances[startNode.hash] = 0
    limitingEdges = {}
    
    queue = [(0, startNode.hash)]
    counter = 0
    while queue:
        counter = counter + 1
        curDistance, curNodeIdx = heapq.heappop(queue)
            
        #add graph ending node when no neighbors
        if len(graph.nodes[curNodeIdx].edges) == 1:
            graph.endNodes[curNodeIdx] = graph.nodes[curNodeIdx]

        
        for edge in (graph.nodes[curNodeIdx].edges):  
                neighbor = edge.endNode
                if neighbor.hash == curNodeIdx:
                    neighbor = edge.startNode
                newDistance = curDistance + edge.length
                if newDistance < distances[neighbor.hash]:
                    #range limit
                    if curDistance <= limit and newDistance > limit:
                        #remaining distance to travel
                        remainingDistance = limit - curDistance
                        switch = False
                        
                        #adjust unit if input is time
                        if graph.unit == 'minutes':
                            remainingDistance = remainingDistance * edge.speed
                            
                        #reverse distance for proper placement of the point on the polyline later
                        if curNodeIdx == edge.endNode.hash:
                            remainingDistance = edge.length - remainingDistance
                            switch = True
                            
                        limitingEdges[edge.fid] = {'dist' : remainingDistance, 'switch' : switch}
                        break
                    distances[neighbor.hash] = newDistance
                    
                    #push with new priority
                    heapq.heappush(queue, (newDistance, neighbor.hash))

    return limitingEdges, graph.endNodes
    
def addToDisplay(filepath, level = 'BOTTOM'):
    mxd = arcpy.mapping.MapDocument('CURRENT')  
    df = arcpy.mapping.ListDataFrames(mxd, 'Layers')[0]
    layer = arcpy.mapping.Layer(filepath)
    arcpy.mapping.AddLayer(df, layer, level)
    arcpy.RefreshActiveView()  
    arcpy.RefreshTOC()

def createArea(inFeatures, limitingEdges, endNodes, range, unit):
    #creates limit points on the limiting edges (based on remaining distances)
    #computes centroid of limit points
    #connects points in order of azimuth between centroid and Nth point
    #smoothens polygon with Bezier interpolation algorithm
    #adds graph ending points which were reached without limit
    def azimuth(center_x, center_y, x, y):
        angle = degrees(atan2(y - center_y, x - center_x))
        az = (angle + 360) % 360
        return az
        
    #select all limiting edges
    condition = ''
    for fid in limitingEdges:
        condition = condition + str(fid) + ','
        
    condition = '"FID" IN (' + condition.rstrip(',') + ')'
    arcpy.SelectLayerByAttribute_management(inFeatures, 'NEW_SELECTION',condition)
          
    pointList = []
    with arcpy.da.SearchCursor(inFeatures, ['FID', 'SHAPE@', 'LENGTH']) as cur:
        xSum = 0
        ySum = 0
        for row in cur:
            fid = row[0]
            polyline = row[1]
            length = row[2]
            dist = limitingEdges[fid]['dist']
            endsSwitched = limitingEdges[fid]['switch']
            #move point along the edge by the remaining distance value
            rangePoint = polyline.positionAlongLine(dist)
            xSum = xSum + rangePoint[0].X
            ySum = ySum + rangePoint[0].Y
            
            #if ends were switched the remaining distance is reversed for "rDistance" point attribute value
            if endsSwitched:
                dist = length - dist
            pointList.append([0, dist, (rangePoint)])
    centroid = (xSum / len(pointList), ySum / len(pointList))
    
    i = 0
    while i < len(pointList):
        p = pointList[i]
        az = azimuth(centroid[0], centroid[1], p[2][0].X, p[2][0].Y)
        pointList[i][0] = az
        i += 1
    
    arcpy.SelectLayerByAttribute_management(inFeatures, 'CLEAR_SELECTION')            
         
    #create rangePoints layer
    suffix = '__' + str(range) + '_' + unit
    print('Creating limit points')
    filename = 'rangePoints' + suffix + '.shp'
    path = arcpy.env.workspace 
    filepath = os.path.join(path, filename)
    arcpy.CreateFeatureclass_management(path, filename, 'POINT')
    arcpy.AddField_management(filepath, 'rDistance')
    arcpy.AddField_management(filepath, 'azimuth')
    cursor = arcpy.da.InsertCursor(filepath,['azimuth', 'rDistance', 'SHAPE@XY'])
    for row in pointList:
        cursor.insertRow(row)
    del cursor
    
    #connect points ordered by azimuth
    inFeatures = filepath
    filename = 'rangeLines' + suffix
    filepath = os.path.join(path, filename)
    arcpy.PointsToLine_management(inFeatures, filepath, Sort_Field = 'azimuth', Close_Line = 'CLOSE')
    arcpy.Delete_management(inFeatures)

    #closed polyline to polygon
    inFeatures = filepath + '.shp'
    filename = 'rangeArea' + suffix + '.shp'
    filepath = os.path.join(path, filename)
    arcpy.FeatureToPolygon_management(inFeatures, filepath, "", "NO_ATTRIBUTES", "")
    polygon = filepath
    arcpy.Delete_management(inFeatures)
    
    #graph ending points visited
    filename = 'pointsReachedWithoutLimit' + suffix + '.shp'
    print('Creating reached graph ending points: ' + filename)
    path = arcpy.env.workspace
    filepath = os.path.join(path, filename)
    arcpy.CreateFeatureclass_management(path, filename, 'POINT')
    cursor = arcpy.da.InsertCursor(filepath,['SHAPE@XY'])
    for id in endNodes:
        x = endNodes[id].x
        y = endNodes[id].y
        newPoint = arcpy.Point(x, y)
        az = azimuth(centroid[0], centroid[1], x, y)
        cursor.insertRow([arcpy.PointGeometry(newPoint)])
    del cursor
    
    #smoothen polygon
    inFeatures = polygon
    filename = 'smoothArea' + suffix + '.shp'
    print('Creating smooth polygon: ' + filename)
    filepath = os.path.join(path, filename)
    arcpy.cartography.SmoothPolygon(inFeatures, filepath, 'BEZIER_INTERPOLATION', 0)
    smoothPolygon = filepath
    
    #check if polygon smoothened correctly, display normal polygon if it did not
    with arcpy.da.SearchCursor(filepath, ['SHAPE@LENGTH']) as cursor:
        for row in cursor:
            if row[0] == None:
                arcpy.Delete_management(smoothPolygon)
            else:
                arcpy.Delete_management(polygon)
            break
    arcpy.RefreshActiveView()
    arcpy.RefreshTOC()