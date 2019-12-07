import arcpy
import heapq
import os
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.hash = 'X' + format(self.x, '.3f')+ 'Y' + format(self.y, '.3f')
        self.edges = []
    
    def toString(self):
        return 'Node: \n\t idx: ' + self.hash + '\n\t cords: \n\t\t[' + str(self.x) + ', ' + str(self.y) + ']'

class Edge:
    def __init__(self, fid, startNode, endNode, length, cost = 1):
        self.startNode = startNode
        self.endNode = endNode
        self.length = length
        self.cost = cost
        self.hash = str(startNode.x) + str(startNode.y) + str(endNode.x) + str(endNode.y)
        self.fid = fid
        
    
    def toString(self):
        return 'Edge: \n\t nodes: \n\t\t[' + self.startNode.hash + ', ' + self.endNode.hash + '] ' + '\n\t length: ' + str(self.length) + '\n\t cost: ' + str(self.cost)
        

class Graph:
    edges = {}    #hash tables
    nodes = {}
            
    def __init__(self, inFeatures):
        repeatingNodes = 0
        repeatingEdges = 0
        cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
            startNode = Node(row.start_x, row.start_y)
            endNode = Node(row.end_x, row.end_y)
            newEdge = Edge(row.fid, startNode, endNode, row.length, 1)
            
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
        
def dijkstra(graph, startNodeId, limit):    #return dictionary {limitingEdgeFid : remainingLength}

    startNode = graph.nodes[startNodeId]
    arcpy.AddMessage('Start node: \n\t' + startNode.toString())
    distances = {node: float('inf') for node in graph.nodes}
    distances[startNode.hash] = 0
    limitingEdges = {}
    
    queue = [(0, startNode.hash)]
    counter = 0
    while len(queue) > 0:
        counter = counter + 1
        curDistance, curNodeIdx = heapq.heappop(queue)
        
        #range limit, break loop after reaching
        if curDistance > limit:
            break
            
        for edge in (graph.nodes[curNodeIdx].edges):  
                neighbor = edge.endNode
                if neighbor.hash == curNodeIdx:
                    neighbor = edge.startNode
                newDistance = curDistance + edge.length
                if newDistance < distances[neighbor.hash]:
                    if curDistance <= limit and newDistance > limit:
                        remainingDistance = limit - curDistance
                        if curNodeIdx == edge.endNode.hash:
                            remainingDistance = edge.length - remainingDistance
                        limitingEdges[edge.fid] = remainingDistance
                        #arcpy.AddMessage('edge length: ' + str(edge.length) + ', remaining dist: ' + str(remainingDistance))
                        break
                    distances[neighbor.hash] = newDistance
                    heapq.heappush(queue, (newDistance, neighbor.hash))

    arcpy.AddMessage('steps: ' + str(counter))

    # for fid in limitingEdges:
        # arcpy.AddMessage('fid: ' + str(fid) + ', remaining distance: ' + str(limitingEdges[fid]))
        
    notFound = 0
    for key in distances:
        if distances[key] == float('inf'):
            notFound = notFound + 1
        else:
            None
            #arcpy.AddMessage(str(key) + ', dist:' + str(distances[key]))
    arcpy.AddMessage(str(notFound) + '/' + str(len(graph.nodes)) + ' nodes not reached')
    return limitingEdges
    
class Toolbox(object):
    def __init__(self):
        self.label =  "Routing toolbox"
        self.alias  = "routing"

        # List of tool classes associated with this toolbox
        self.tools = [PrepareData, RangeFromPoint]


class PrepareData(object):
#adds geometry attributes needed for graph
    def __init__(self):
        self.label       = "Prepare data"
        self.description = "Creates a layer of start/destination points, adds necessary geometry attributes to the input polyline layer."

    def getParameterInfo(self):
        #Define parameter definitions

        # Input Features parameter
        inputRoads = arcpy.Parameter(
            displayName="Input roads layer",
            name="inputRoads",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        
        inputRoads.filter.list = ["Polyline"]
        
        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [inputRoads.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [inputRoads, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True

    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        inFeatures  = parameters[0].valueAsText
        outFeatures = "points.shp"
        arcpy.overwriteoutput = True
        # arcpy.AddMessage("FeatureVerticesToPoints (" + inFeatures + ")")
        # arcpy.FeatureVerticesToPoints_management(inFeatures, outFeatures, "ALL")
        arcpy.AddMessage("AddGeometryAttributes (" + inFeatures + ")")
        arcpy.AddGeometryAttributes_management(inFeatures, ["LENGTH","LINE_START_MID_END"])

class RangeFromPoint(object):
    #creates graph, uses dijkstra algorithm
    def __init__(self):
        self.label       = "Range from point"
        self.description = "Returns an area which can be reached within given time/distance from a starting point"

    def getParameterInfo(self):
        #Define parameter definitions
        
        startPoint = arcpy.Parameter(
            displayName="Input Features",
            name="startPoint",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        startPoint.filter.list = ["Polyline"]

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
        
        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [startPoint.name, range.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [startPoint, range, rangeUnit, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True
        
    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        arcpy.overwriteoutput = True
        inFeatures  = parameters[0].valueAsText
        range   = int(parameters[1].valueAsText)
        unit = parameters[2].valueAsText

        arcpy.AddMessage("Creating graph from layer: " + inFeatures)
        newGraph = Graph(inFeatures)
        arcpy.AddMessage(newGraph.toString())
		
        startNodeId = 'X473669.390Y572807.990'
        limitingEdges = dijkstra(newGraph, startNodeId, range)
        
        condition = ''
        for fid in limitingEdges:
            condition = condition + str(fid) + ','
        condition = '"FID" IN (' + condition.rstrip(',') + ')'
        arcpy.AddMessage('select * from ' + inFeatures + ' where ' + condition)
        arcpy.SelectLayerByAttribute_management
        (
            inFeatures, 
            'NEW_SELECTION', 
            condition
        )
        
        path = arcpy.env.workspace
        file = os.path.join(path, 'range__' + str(range))
        arcpy.AddMessage('writing to file: ' + file)
        arcpy.MinimumBoundingGeometry_management
        (
            inFeatures, 
            file, 
            'CONVEX_HULL', 
            'ALL'
        )
            
        arcpy.SelectLayerByAttribute_management(inFeatures, "CLEAR_SELECTION")
        
        #display bounding geometry
        mxd = arcpy.mapping.MapDocument('CURRENT')  
        df = arcpy.mapping.ListDataFrames(mxd, 'Layers')[0]
        addLayer = arcpy.mapping.Layer(file)  
        arcpy.mapping.AddLayer(df, addLayer, 'BOTTOM')  
        arcpy.RefreshActiveView()  
        arcpy.RefreshTOC()
        
        #to do
        #get last points in range
        #add the remaining edge length to create new points
        #connect points

        # condition = ''
        # outFeatures = 'range.shp'
        # for fid in limitingEdges:
            # condition = '"FID" = ' + str(fid)
            # remainingDistance = str(limitingEdges[fid]) + ' meters'
            # arcpy.SelectLayerByAttribute_management("L4_1_BDOT10k__OT_SKDR_L", "NEW_SELECTION", condition)
            # arcpy.GeneratePointsAlongLines_management(inFeatures, outFeatures, 'DISTANCE', Distance=remainingDistance)