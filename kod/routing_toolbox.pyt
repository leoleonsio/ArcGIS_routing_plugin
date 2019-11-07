import arcpy
#import graph

class Toolbox(object):
    def __init__(self):
		self.label =  "Sinuosity toolbox"
		self.alias  = "sinuosity"

        # List of tool classes associated with this toolbox
		#self.tools = [CalculateRoute]	#returns route from point given point to other given point
		#self.tools = [EquidistantPoints]	#returns set of points with equal route length to a given point
		self.tools = [CalculateRoute, EquidistantPoints]

class CalculateRoute(object):
    def __init__(self):
        self.label       = "Calculate Route"
        self.description = "Returns route from point given point to other given point"

    def getParameterInfo(self):
        #Define parameter definitions

        # Input Features parameter
        startPoint = arcpy.Parameter(
            displayName="Start point",
            name="startPoint",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        startPoint.filter.list = ["Multipoint"]
		
	endPoint = arcpy.Parameter(
	    displayName="End point",
	    name="endPoint",
	    datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        endPoint.filter.list = ["Multipoint"]
		
	roads = arcpy.Parameter(
	    displayName="Roads layer",
	    name="roads",
	    datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        roads.filter.list = ["Polyline"]

        # Route type parameter
        routeType = arcpy.Parameter(
            displayName="Route type",
            name="routeType",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        routeType.filter.list = ['Fastest', 'Shortest']
        routeType.value = 'fastest'     
        #routeType.enabled = 0	
		
		# Route cost parameter
        isCostFree = arcpy.Parameter(
            displayName="Cost free",
            name="isCostFree",
            datatype="Boolean",
            parameterType="Required",
            direction="Input")
			
        isCostFree.value = 0

        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [startPoint.name, endPoint.name]
	#outFeatures.parameterDependencies = [endPoint.name]
        outFeatures.schema.clone = True

        parameters = [startPoint, endPoint, roads, routeType, isCostFree, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True

    # def updateParameters(self, parameters): #optional
        # if parameters[0].altered:
            # parameters[1].value = arcpy.ValidateFieldName(parameters[1].value,
                                                          # parameters[0].value)
        # return

    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        startPoint = parameters[0].valueAsText
	endPoint = parameters[1].valueAsText
        roads = parameters[2].valueAsText
	routeType = parameters[3].valueAsText
	isCostFree = parameters[4].valueAsText


class EquidistantPoints(object):
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
        routeType = arcpy.Parameter(
            displayName="Sinuosity Field",
            name="routeType",
            datatype="Field",
            parameterType="Optional",
            direction="Input")
        
        routeType.value = "sinuosity"
        
        # Derived Output Features parameter
        outFeatures = arcpy.Parameter(
            displayName="Output Features",
            name="outFeatures",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        outFeatures.parameterDependencies = [startPoint.name]
        outFeatures.schema.clone = True

        parameters = [startPoint, routeType, outFeatures]
        
        return parameters

    def isLicensed(self): #optional
        return True

    def updateParameters(self, parameters): #optional
        if parameters[0].altered:
            parameters[1].value = arcpy.ValidateFieldName(parameters[1].value,
                                                          parameters[0].value)
        return

    def updateMessages(self, parameters): #optional
        return

    def execute(self, parameters, messages):
        inFeatures  = parameters[0].valueAsText
        fieldName   = parameters[1].valueAsText
        
        if fieldName in ["#", "", None]:
            fieldName = "sinuosity"

        arcpy.AddField_management(inFeatures, fieldName, 'DOUBLE')

        expression = '''
import math
def getSinuosity(shape):
    length = shape.length
    d = math.sqrt((shape.firstPoint.X - shape.lastPoint.X) ** 2 +
                  (shape.firstPoint.Y - shape.lastPoint.Y) ** 2)
    return d/length
'''

        arcpy.CalculateField_management(inFeatures,
                                        fieldName,
                                        'getSinuosity(!shape!)',
                                        'PYTHON_9.3',
                                        expression)