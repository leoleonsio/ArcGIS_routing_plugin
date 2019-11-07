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
        in_features = arcpy.Parameter(
            displayName="Input Features",
            name="in_features",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        
        in_features.filter.list = ["Polyline"]

        # Route type parameter
        route_type_field = arcpy.Parameter(
            displayName="Route type",
            name="route_type_field",
            datatype="GPString",
            parameterType="Required",
            direction="Input")
        route_type_field.filter.list = ['Fastest', 'Shortest']
        route_type_field.value = 'fastest'     
        #route_type_field.enabled = 0	
		
		# Route cost parameter
        is_cost_free = arcpy.Parameter(
            displayName="Cost free",
            name="is_cost_free",
            datatype="Boolean",
            parameterType="Required",
            direction="Input")
			
        is_cost_free.value = 0

        # Derived Output Features parameter
        out_features = arcpy.Parameter(
            displayName="Output Features",
            name="out_features",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        out_features.parameterDependencies = [in_features.name]
        out_features.schema.clone = True

        parameters = [in_features, route_type_field, is_cost_free, out_features]
        
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
class EquidistantPoints(object):
    def __init__(self):
        self.label       = "Equidistant points"
        self.description = "Returns set of points with equal route length to a given point"

    def getParameterInfo(self):
        #Define parameter definitions

        # Input Features parameter
        in_features = arcpy.Parameter(
            displayName="Input Features",
            name="in_features",
            datatype="GPFeatureLayer",
            parameterType="Required",
            direction="Input")
        
        in_features.filter.list = ["Polyline"]

        # Sinuosity Field parameter
        route_type_field = arcpy.Parameter(
            displayName="Sinuosity Field",
            name="route_type_field",
            datatype="Field",
            parameterType="Optional",
            direction="Input")
        
        route_type_field.value = "sinuosity"
        
        # Derived Output Features parameter
        out_features = arcpy.Parameter(
            displayName="Output Features",
            name="out_features",
            datatype="GPFeatureLayer",
            parameterType="Derived",
            direction="Output")
        
        out_features.parameterDependencies = [in_features.name]
        out_features.schema.clone = True

        parameters = [in_features, route_type_field, out_features]
        
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