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
        self.hash = startNode.hash + endNode.hash
        self.fid = fid
        
    
    def toString(self):
        return 'Edge: \n\t nodes: \n\t\t[' + self.startNode.hash + ', ' + self.endNode.hash + '] ' + '\n\t length: ' + str(self.length) + '\n\t cost: ' + str(self.cost)
        

class Graph:
    edges = {}    #hash tables
    nodes = {}
    def __init__(self, inFeatures):
        repeatingNodes = 0
        repeatingEdges = 0
	cost = {"A":140,"D":30,"G":50,"GP":100,"I":50,"L":50,"S":120,"Z":50}
	cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
            startNode = Node(row.start_x, row.start_y)
            endNode = Node(row.end_x, row.end_y)
            newEdge = Edge(row.fid, startNode, endNode, row.length, cost[row.klasaDrogi]*(1000/60))
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

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def heuristic(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
	
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional	
    return path
	
def a_star_search(graph, startPoint, endPoint):
    frontier = PriorityQueue()
    frontier.put(startPoint, 0)
    came_from = {}
    cost_so_far = {}
    came_from[startPoint] = None
    cost_so_far[startPoint] = 0
    
    while not frontier.empty():
        current = frontier.get()
    
	if current == endPoint:
            break
        

				
	for edge in (graph.nodes[current].edges):
            next = edge.endNode.hash
            new_cost = cost_so_far[current] + edge.length
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(graph.nodes[endPoint], graph.nodes[next])
                frontier.put(next, priority)
                came_from[next] = current
    
    path = reconstruct_path(came_from, startPoint, endPoint)
    
    ret = []
    distance = 0
    for i in range(0,len(path)-1):
        ret.append(graph.edges[path[i]+path[i+1]].fid)
        distance+= graph.edges[path[i]+path[i+1]].length
	
    return ret,distance
	
class Toolbox(object):
    def __init__(self):
        self.label =  "Routing toolbox"
        self.alias  = "routing"

        # List of tool classes associated with this toolbox
        self.tools = [PrepareData, Astar]


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

class Astar(object):
    #creates graph, uses astar algorithm
    def __init__(self):
        self.label       = "Astar"
        self.description = "Returns fastest/shortest route between two points"

    def getParameterInfo(self):
        #Define parameter definitions
        
        startPoint = arcpy.Parameter(
            displayName="Input Features",
            name="startPoint",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        startPoint.filter.list = ["Polyline"]
        
        astartype = arcpy.Parameter(
            displayName="AstarType",
            name="astartype",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        astartype.filter.list = ['shortest', 'fastest']
        astartype.value = 'shortest'
        
        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [startPoint.name, astartype.name, outFeatures.name]
        outFeatures.schema.clone = True

        parameters = [startPoint, astartype, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True
        
    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        #arcpy.overwriteoutput = True
        inFeatures  = parameters[0].valueAsText
        type = parameters[1].valueAsText

        arcpy.AddMessage("Creating graph from layer: " + inFeatures)
        newGraph = Graph(inFeatures)
        arcpy.AddMessage(newGraph.toString())
		
        startNodeId = 'X473669.390Y572807.990'
        endNodeId = 'X480816.140Y575680.120'
        astarpath,distance = a_star_search(newGraph,startNodeId,endNodeId)
        arcpy.AddMessage('Distance to travel:' +str(distance))
        condition = ''
        for fid in astarpath:
            condition = condition + str(fid) + ','
        condition = '"FID" IN (' + condition.rstrip(',') + ')'
        arcpy.AddMessage('select * from ' + inFeatures + ' where ' + condition)
        arcpy.SelectLayerByAttribute_management(inFeatures, 'NEW_SELECTION',condition)
		
        