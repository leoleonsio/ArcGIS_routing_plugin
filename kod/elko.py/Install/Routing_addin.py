import arcpy
import pythonaddins
import os
import math
import heapq
import datetime


class EndPointSelection(object):
    """Implementation for endpoint_tool (Tool)"""

    def __init__(self):
        self.enabled = False
        self.first = True
        self.shape = "NONE"  

    def onMouseDown(self, x, y, button, shift):
        pass

    def onMouseDownMap(self, x, y, button, shift):
        # make sure layer exists
        fc = "./endpoint.shp"
        if self.first:
            check(fc, 'endpoint.shp')
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
        near_features = layer_combobox.value
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
                routing_type_combobox.value = '' 
                routing_type_combobox.refresh()
                path_button_button.enabled = False
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
        routing_type_combobox.enabled = True

    def onMouseUp(self, x, y, button, shift):
        pass

    def onMouseUpMap(self, x, y, button, shift):
        pass

    def onMouseMove(self, x, y, button, shift):
        pass

    def onMouseMoveMap(self, x, y, button, shift):
        pass

    def onDblClick(self):
        pass

    def onKeyDown(self, keycode, shift):
        pass

    def onKeyUp(self, keycode, shift):
        pass

    def deactivate(self):
        pass

    def onCircle(self, circle_geometry):
        pass

    def onLine(self, line_geometry):
        pass

    def onRectangle(self, rectangle_geometry):
        pass


class StartPointSelection(object):
    """Implementation for startpoint_tool (Tool)"""

    def __init__(self):
        self.enabled = False
        self.first = True
        self.shape = "NONE"  

    def onMouseDown(self, x, y, button, shift):
        pass

    def onMouseDownMap(self, x, y, button, shift):
        # make sure layer exists
        fc = "./startpoint.shp"
        if self.first:
            check(fc, 'startpoint.shp')
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
        endpoint_tool.enabled = True

    def onMouseUp(self, x, y, button, shift):
        pass

    def onMouseUpMap(self, x, y, button, shift):
        pass

    def onMouseMove(self, x, y, button, shift):
        pass

    def onMouseMoveMap(self, x, y, button, shift):
        pass

    def onDblClick(self):
        pass

    def onKeyDown(self, keycode, shift):
        pass

    def onKeyUp(self, keycode, shift):
        pass

    def deactivate(self):
        pass

    def onCircle(self, circle_geometry):
        pass

    def onLine(self, line_geometry):
        pass

    def onRectangle(self, rectangle_geometry):
        pass

class layerselection(object):
    """Implementation for Routing_addin.layer_combobox (ComboBox)"""
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
        startpoint_tool.enabled = True
        if len(arcpy.ListFields(self.layerpath,'START_X'))>0:
            print('Data already prepared')
        else:
            print('Preparing data...')
            arcpy.AddGeometryAttributes_management(self.layerpath, ["LENGTH","LINE_START_MID_END"])
            print('Data prepared')
    def onEditChange(self, text):
        pass
    def onFocus(self, focused):
        if focused:  
            self.mxd = arcpy.mapping.MapDocument('current')  
            self.df = self.mxd.activeDataFrame  
            lyrs = [l.name for l in arcpy.mapping.ListLayers(self.mxd, '*', self.df)  
                    if not l.isBroken and l.isFeatureLayer and  
                    arcpy.Describe(l).shapeType == 'Polyline']  
            self.items = sorted(lyrs)
    def onEnter(self):
        pass
    def refresh(self):
        pass
        
class path_button(object):
    """Implementation for path_button_button (Button)"""

    def __init__(self):
        self.enabled = False
        self.checked = False

    def onClick(self):

        arcpy.env.overwriteOutput = True
        # prepdata = False
        inFeatures = layer_combobox.layerpath
        type = routing_type_combobox.value

        cursor = arcpy.da.UpdateCursor('./startpoint.shp', ["SHAPE@XY"])
        for row in cursor:
            xstart = row[0][0]
            ystart = row[0][1]
        cursor = arcpy.da.UpdateCursor('./endpoint.shp', ["SHAPE@XY"])
        for row in cursor:
            xend = row[0][0]
            yend = row[0][1]

        # if prepdata == 'true':
        #    arcpy.AddGeometryAttributes_management(inFeatures, ["LENGTH","LINE_START_MID_END"])

        print("Creating " + type + " route")
        # print("Creating graph from layer: " + inFeatures)

        newGraph = Graph(inFeatures)

        startNodeId = 'X' + format(xstart, '.1f') + 'Y' + format(ystart, '.1f')
        endNodeId = 'X' + format(xend, '.1f') + 'Y' + format(yend, '.1f')

        # startNodeId = 'X473669.390Y572807.990'
        # endNodeId = 'X480816.140Y575680.120'
        # endNodeId = 'X473669.4Y572808.0'
        # startNodeId = 'X480816.1Y575680.1'
        astarpath, distance, time = a_star_search(newGraph, startNodeId, endNodeId, type)
        if astarpath == distance == time == -1:
            return
        print('Distance(m): ' + distance)
        print('Time(hh:mm:ss): ' +  str(datetime.timedelta(minutes=float(time))))
        condition = ''
        for fid in astarpath:
            condition = condition + str(fid) + ','
        condition = '"FID" IN (' + condition.rstrip(',') + ')'
        arcpy.FeatureClassToFeatureClass_conversion(inFeatures, "./", "route.shp", condition)
        mxd = arcpy.mapping.MapDocument("CURRENT")
        df = arcpy.mapping.ListDataFrames(mxd, "*")[0]
        newlayer = arcpy.mapping.Layer('./route.shp')
        arcpy.mapping.AddLayer(df, newlayer, "TOP")
        arcpy.RefreshActiveView()
        arcpy.RefreshTOC()


class routing_type(object):
    """Implementation for routing_type_combobox (ComboBox)"""

    def __init__(self):
        self.items = ["shortest", "fastest"]
        self.editable = True
        self.enabled = False
        self.dropdownWidth = '123456789'
        self.width = '123456789'

    def onSelChange(self, selection):
        path_button_button.enabled = True
        self.value = selection
        self.refresh()

    def onEditChange(self, text):
        pass

    def onFocus(self, focused):
        pass

    def onEnter(self):
        pass

    def refresh(self):
        pass


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
        self.x = round(x, 1)  # small round to correct minimal input data errors
        self.y = round(y, 1)
        self.hash = 'X' + format(self.x, '.1f') + 'Y' + format(self.y, '.1f')
        self.edges = []

    def toString(self):
        return 'Node: \n\t idx: ' + self.hash + '\n\t cords: \n\t\t[' + str(self.x) + ', ' + str(self.y) + ']'


class Edge:
    def __init__(self, fid, startNode, endNode, length, cost):
        self.startNode = startNode
        self.endNode = endNode
        self.length = length
        self.cost = cost
        self.hash = startNode.hash + endNode.hash
        self.fid = fid

    def toString(self):
        return 'Edge: \n\t nodes: \n\t\t[' + self.startNode.hash + ', ' + self.endNode.hash + '] ' + '\n\t length: ' + str(
            self.length) + '\n\t cost: ' + str(self.cost)


class Graph:
    edges = {}
    nodes = {}
    repeatingNodes = 0
    repeatingEdges = 0
    def __init__(self, inFeatures):
        cost = {"A": 140, "D": 30, "G": 50, "GP": 100, "I": 50, "L": 50, "S": 120, "Z": 50}
        repeatingNodes = 0
        repeatingEdges = 0
        cursor = arcpy.SearchCursor(inFeatures)
        for row in cursor:
            startNode = Node(row.start_x, row.start_y)
            endNode = Node(row.end_x, row.end_y)
            newEdge = Edge(row.fid, startNode, endNode, row.length, row.length / (cost[row.klasaDrogi] * (1000 / 60)))

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

        # print('repeating nodes: ' + str(repeatingNodes) + '\nrepeating edges: ' + str(repeatingEdges))

    def containsNode(self, node):
        if node.hash in self.nodes:
            return 1
        return 0

    def containsEdge(self, edge):
        if edge.hash in self.edges:
            return 1
        return 0

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


def length_heuristic(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def time_heuristic(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2) / (140 * (1000 / 60))


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def a_star_search(graph, startPoint, endPoint, type):
    starttime = datetime.datetime.now()
    frontier = PriorityQueue()
    frontier.put(startPoint, 0)
    came_from = {}
    cost_so_far = {}
    came_from[startPoint] = None
    cost_so_far[startPoint] = 0
    counter = 0
    ecounter = 0
    if type == 'shortest':
        while not frontier.empty():
            counter+=1
            current = frontier.get()
            if current == endPoint:
                break
            for edge in graph.nodes[current].edges:
                if edge.endNode.hash != current:
                    next = edge.endNode.hash
                else:
                    next = edge.startNode.hash
                new_cost = cost_so_far[current] + edge.length
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    ecounter+=1
                    cost_so_far[next] = new_cost
                    priority = new_cost + length_heuristic(graph.nodes[endPoint], graph.nodes[next])
                    frontier.put(next, priority)
                    came_from[next] = current
            if frontier.empty():
                print('Brak polaczenia miedzy punktami')
                return -1,-1,-1
    elif type == 'fastest':
        while not frontier.empty():
            counter+=1
            current = frontier.get()
            if current == endPoint:
                break
            for edge in graph.nodes[current].edges:
                if edge.endNode.hash != current:
                    next = edge.endNode.hash
                else:
                    next = edge.startNode.hash
                new_cost = cost_so_far[current] + edge.cost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    ecounter+=1
                    cost_so_far[next] = new_cost
                    priority = new_cost + time_heuristic(graph.nodes[endPoint], graph.nodes[next])
                    frontier.put(next, priority)
                    came_from[next] = current
            if frontier.empty():
                print('Brak polaczenia miedzy punktami')
                return -1,-1,-1
    else:
        print('problem with routing type')
    endtime = datetime.datetime.now()
    time = endtime - starttime
    print('Algorithm time: ' + str(time))
    print('Number of nodes traversed: ' + str(counter) +'/'+ str(len(graph.nodes)))
    print('Number of edges traversed: ' + str(ecounter) +'/'+ str(len(graph.edges)))
    path = reconstruct_path(came_from, startPoint, endPoint)

    ret = []
    distance = 0
    time = 0
    for i in range(0, len(path) - 1):
        if path[i] + path[i + 1] in graph.edges:
            ret.append(graph.edges[path[i] + path[i + 1]].fid)
            distance += graph.edges[path[i] + path[i + 1]].length
            time += graph.edges[path[i] + path[i + 1]].cost
        else:
            ret.append(graph.edges[path[i + 1] + path[i]].fid)
            distance += graph.edges[path[i + 1] + path[i]].length
            time += graph.edges[path[i + 1] + path[i]].cost
    return ret, format(distance, '.1f'), format(time, '.1f')
