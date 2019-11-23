import arcpy

class Toolbox(object):
    def __init__(self):
		self.label =  "Sinuosity toolbox"
		self.alias  = "sinuosity"

        # List of tool classes associated with this toolbox
		self.tools = [PrepareData, EquidistantPoints]


class PrepareData(object):
#dodaje atrybuty długości oraz wierzchołków linii, tworzy warstwę punktową, z której później będzie można wybierać punkty początkowe/końcowe
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
	#tworzy graf, liczy zasięgi od danego punktu
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
		##Zrobi graf, wyznaczy zasiegi, doda do widoku
        g = Graph(inFeatures)