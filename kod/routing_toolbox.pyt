import arcpy
import heapq
import os
from math import sqrt, degrees, atan2
import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = round(x, 1)    #small round to correct minimal input data errors
        self.y = round(y, 1)
        self.hash = 'X' + format(self.x, '.3f')+ 'Y' + format(self.y, '.3f')
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
            
        arcpy.AddMessage('repeating nodes: ' + str(repeatingNodes) + '\nrepeating edges: ' + str(repeatingEdges))
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
             
def dijkstra(graph, startNodeId, limit):    #returns dictionary of limiting edges

    startNode = graph.nodes[startNodeId]
    arcpy.AddMessage('Start node: \n\t' + startNode.toString())
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
            # edge = graph.nodes[curNodeIdx].edges[0]
            # boundingNode = edge.endNode
            # if boundingNode.hash == curNodeIdx:
                # boundingNode = edge.startNode
                #switch
                #edge length
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
                            
                        #reverse distances for proper placement of the point on the polyline
                        if curNodeIdx == edge.endNode.hash:
                            remainingDistance = edge.length - remainingDistance
                            switch = True
                            
                        limitingEdges[edge.fid] = {'dist' : remainingDistance, 'switch' : switch}
                        break
                    distances[neighbor.hash] = newDistance
                    
                    #push with new priority
                    heapq.heappush(queue, (newDistance, neighbor.hash))

    arcpy.AddMessage('steps: ' + str(counter))

    #disp limiting edges
    # for fid in limitingEdges:
        # arcpy.AddMessage('fid: ' + str(fid) + ', remaining distance: ' + str(limitingEdges[fid]))
        
    #disp nodes and their distances
    # notFound = 0
    # for key in distances:
        # if distances[key] == float('inf'):
            # notFound = notFound + 1
        # else:
            # arcpy.AddMessage(str(key) + ', dist:' + str(distances[key]))
    # arcpy.AddMessage(str(notFound) + '/' + str(len(graph.nodes)) + ' nodes not reached')
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
    #disp condition
    #arcpy.AddMessage(condition)
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
            
            #if ends were switched the remaining distance is reversed for "remainingDistance" point attribute value
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
    arcpy.AddMessage('Creating limit points')
    filename = 'rangePoints' + suffix
    path = arcpy.env.workspace
    filepath = os.path.join(path, filename)
    arcpy.CreateFeatureclass_management(path, filename, 'POINT')
    arcpy.AddField_management(filepath, 'remainingDistance')
    arcpy.AddField_management(filepath, 'azimuth')
    cursor = arcpy.da.InsertCursor(filepath,['azimuth', 'remainingDistance', 'SHAPE@XY'])
    for row in pointList:
        cursor.insertRow(row)
    del cursor
    #disp range limit points
    #addToDisplay(filepath)
    
    #connect points ordered by azimuth
    arcpy.AddMessage('Creating range polygon')
    inFeatures = filepath
    filename = 'rangeLines' + suffix
    filepath = os.path.join(path, filename)
    arcpy.PointsToLine_management(inFeatures, filepath, Sort_Field = 'azimuth', Close_Line = 'CLOSE')
    #disp connected points
    #addToDisplay(filepath)
   
    #closed polyline to polygon
    inFeatures = filepath
    filename = 'rangeArea' + suffix
    filepath = os.path.join(path, filename)
    arcpy.FeatureToPolygon_management(inFeatures, filepath, "", "NO_ATTRIBUTES", "")
    polygon = filepath
    
    #graph ending points visited
    filename = 'pointsReachedWithoutLimit' + suffix
    arcpy.AddMessage('Creating reached graph ending points: ' + filename)
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
    addToDisplay(filepath)
    
    #smoothen polygon
    inFeatures = polygon
    filename = 'smoothArea' + suffix
    arcpy.AddMessage('Creating smooth polygon: ' + filename)
    filepath = os.path.join(path, filename)
    arcpy.cartography.SmoothPolygon(inFeatures, filepath, 'BEZIER_INTERPOLATION', 0)
    #disp smooth polygon
    #addToDisplay(filepath)
    smoothPolygon = filepath
    
    #check if polygon smoothened correctly, display normal polygon if it did not
    with arcpy.da.SearchCursor(filepath, ['Shape_Length']) as cursor:
        for row in cursor:
            if row[0] == None:
                addToDisplay(polygon)
            else:
                addToDisplay(smoothPolygon)
            break
            
def createPoint(pointId, range, unit):
    x, y = pointId.replace('X','').split('Y')
    point = arcpy.Point(float(x), float(y))
    
    suffix = '__' + str(range) + '_' + unit 
    filename = 'startPoint' + suffix
    path = arcpy.env.workspace
    filepath = os.path.join(path, filename)
    arcpy.CopyFeatures_management(arcpy.PointGeometry(point), filepath)
    
    #disp point
    addToDisplay(filepath, 'TOP')
        
class Toolbox(object):
    def __init__(self):
        self.label =  "Routing toolbox"
        self.alias  = "routing"

        # List of tool classes associated with this toolbox
        self.tools = [PrepareData, RangeFromPoint]

class PrepareData(object):
    #displays graph nodes for later use
    def __init__(self):
        self.label       = "Prepare data"
        self.description = "Adds geometry attributes if needed, displays available nodes for later use"

    def getParameterInfo(self):
        inFeatures = arcpy.Parameter(
            displayName="Input Features",
            name="inFeatures",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        inFeatures.filter.list = ["Polyline"]
        
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [inFeatures.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [inFeatures, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True
        
    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        inFeatures  = parameters[0].valueAsText
        
        fieldsExist = True
        geomFields = ['LENGTH', 'START_X', 'START_Y', 'END_X', 'END_Y']
        for field in geomFields:
            if len(arcpy.ListFields(inFeatures, field)) == 0:
                fieldsExist = False
                arcpy.AddMessage('Field does not exist: ' + field)
                break
            arcpy.AddMessage('Field exists: ' + field)
        if not fieldsExist:
            arcpy.AddMessage("AddGeometryAttributes (" + inFeatures + ")")
            arcpy.AddGeometryAttributes_management(inFeatures, ["LENGTH","LINE_START_MID_END"])
            
        newGraph = Graph(inFeatures)
        arcpy.AddMessage(newGraph.toString())
        for n in newGraph.nodes:
            arcpy.AddMessage(newGraph.nodes[n].toString())
        del newGraph

class RangeFromPoint(object):
    #creates graph, uses dijkstra algorithm
    def __init__(self):
        self.label       = "Range from point"
        self.description = "Returns an area which can be reached within given time/distance from a starting point"

    def getParameterInfo(self):
        roadsLayer = arcpy.Parameter(
            displayName="Input Features",
            name="roadsLayer",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        roadsLayer.filter.list = ["Polyline"]
        
        startPoint = arcpy.Parameter(
            displayName="Start point ID",
            name="startPoint",
            datatype="GPString",
            parameterType="Required",
            direction="Input")

        range = arcpy.Parameter(
            displayName="Range",
            name="range",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        
        range.value = "1500"
        
        rangeUnit = arcpy.Parameter(
            displayName="Unit",
            name="unit",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        rangeUnit.filter.list = ['minutes', 'meters']
        rangeUnit.value = 'meters'
        
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [roadsLayer.name, startPoint.name, range.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [roadsLayer, startPoint, range, rangeUnit, outFeatures]
        return parameters

    def isLicensed(self): #optional
        return True
        
    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        arcpy.overwriteoutput = True
        inFeatures  = parameters[0].valueAsText
        startNodeId = parameters[1].valueAsText
        range   = int(parameters[2].valueAsText)
        unit = parameters[3].valueAsText

        #select all in case arcmap gets confused
        arcpy.SelectLayerByAttribute_management(inFeatures, 'NEW_SELECTION','1 = 1')
        arcpy.AddMessage('Creating graph from layer: ' + inFeatures)
        newGraph = Graph(inFeatures, unit)
        arcpy.AddMessage(newGraph.toString())
        
        #add start point to map
        #createPoint(startNodeId, range, unit)
        
        #obtain and visualize limiting edges
        limitingEdges, endNodes = dijkstra(newGraph, startNodeId, range)
        arcpy.AddMessage('Range edges calculated. Beginning geoprocessing...')
        createArea(inFeatures, limitingEdges, endNodes, range, unit)
        del newGraph
        del limitingEdges