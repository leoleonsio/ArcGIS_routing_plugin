import arcpy
#na razie heapq, do zrobienia kolejka priorytetowa
import heapq

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.hash = 'X' + str(x)+ 'Y' + str(y)
        self.edges = []
    
    def toString(self):
        return 'Node: \n\t idx: ' + self.hash + '\n\t cords: \n\t\t[' + str(self.x) + ', ' + str(self.y) + ']'

class Edge:
    def __init__(self, startNode, endNode, length, cost = 1):
        self.startNode = startNode
        self.endNode = endNode
        self.length = length
        self.cost = cost
    
    def toString(self):
        return 'Edge: \n\t nodes: \n\t\t[' + self.startNode.hash + ', ' + self.endNode.hash + '] ' + '\n\t length: ' + str(self.length) + '\n\t cost: ' + str(self.cost)
        

class Graph:
    edges = []    #list
    nodes = {}    #hash table
    neighbors = {}
            
    def __init__(self, inFeatures):
        #wypełnienie krawędzi
        cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
            startNode = Node(row.start_x, row.start_y)
            endNode = Node(row.end_x, row.end_y)
            newEdge = Edge(startNode, endNode, row.length, 1)
            self.edges.append(newEdge)
            
            #if not self.containsNode(startNode):   
            self.nodes[startNode.hash] = startNode
            self.nodes[startNode.hash].edges.append(newEdge)
            #if not self.containsNode(endNode):
            self.nodes[endNode.hash] = endNode
            self.nodes[endNode.hash].edges.append(newEdge)
            
        #dictionary of sets of neighbors for each node
        self.neighbors = {node: set() for node in self.nodes}
        for edge in self.edges:
            self.neighbors[edge.startNode.hash].add((edge.endNode.hash, edge.length))
        # for i in self.neighbors:
            # arcpy.AddMessage(i + ', neighbors: ' + str(len(self.neighbors[i])))
            
    def containsNode(self, node):
        if node.hash in self.nodes:
            return 1
        return 0;    

    def toString(self):
        return 'Graph: \n\t nodes: ' + str(len(self.nodes)) + ' \n\t edges: ' + str(len(self.edges))
        
#def dijkstra(graph, startNode, inputDistance):
def dijkstra(graph, inputDistance):
    for x in graph.nodes:
        key = x
        break
    startNode = graph.nodes[key]
    arcpy.AddMessage('Start node: \n\t' + startNode.toString())
    distances = {node: float('inf') for node in graph.nodes}
    distances[startNode.hash] = 0
    
    queue = [(0, startNode.hash)]
    while len(queue) > 0:
        curDistance, curNodeIdx = heapq.heappop(queue)
        
        if curDistance > distances[curNodeIdx]:
            continue
            
        for neighbor, length in graph.neighbors[curNodeIdx]:
            distance = curDistance + length
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))
                
    for key in distances:
        arcpy.AddMessage(str(key) + ':' + str(distances[key]))  
        
class Toolbox(object):
    def __init__(self):
        self.label =  "Routing toolbox"
        self.alias  = "routing"

        # List of tool classes associated with this toolbox
        self.tools = [PrepareData, EquidistantPoints]


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

class EquidistantPoints(object):
    #creates graph, uses dijkstra algorithm
    def __init__(self):
        self.label       = "Equidistant points"
        self.description = "Returns set of points with equal route length to a given point"

    def getParameterInfo(self):
        #Define parameter definitions

        # Input Features parameter
        startPoint = arcpy.Parameter(
            displayName="Input Features",
            name="startPoint",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        
        startPoint.filter.list = ["Polyline"]

        # Sinuosity Field parameter
        range = arcpy.Parameter(
            displayName="Range",
            name="range",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        
        range.value = "100"
        
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
        inFeatures  = parameters[0].valueAsText
        range   = parameters[1].valueAsText
        unit = parameters[2].valueAsText

        arcpy.AddMessage("Creating graph from layer: " + inFeatures)
        newGraph = Graph(inFeatures)
        arcpy.AddMessage(newGraph.toString())
		
        dijkstra(newGraph, 50)