from parser import Node

class NavigationInfo(object):
    def __init__(self):
        self.avatarSize = [0.25, 1.6000000000000001, 0.75] #MFFloat
        self.headlight = True #SFBool
        self.speed = 1.0 #SFFloat
        self.type = 'WALK' #MFString
        self.visibilityLimit = 0.0 #SFFloat

class FontStyle(object):
    def __init__(self):
        self.family = 'SERIF' #SFString
        self.horizontal = True #SFBool
        self.justify = 'BEGIN' #MFString
        self.language = '' #SFString
        self.leftToRight = True #SFBool
        self.size = 1.0 #SFFloat
        self.spacing = 1.0 #SFFloat
        self.style = 'PLAIN' #SFString
        self.topToBottom = True #SFBool

class Group(object):
    def __init__(self):
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f

class SphereSensor(object):
    def __init__(self):
        self.autoOffset = True #SFBool
        self.enabled = True #SFBool
        self.offset = [0.0, 1.0, 0.0, 0.0] #SFRotation

class LOD(object):
    def __init__(self):
        self.center = [0.0, 0.0, 0.0] #SFVec3f
        self.level = [] #MFNode
        self.range = [] #MFFloat

class PlaneSensor(object):
    def __init__(self):
        self.autoOffset = True #SFBool
        self.enabled = True #SFBool
        self.maxPosition = [-1.0, -1.0] #SFVec2f
        self.minPosition = [0.0, 0.0] #SFVec2f
        self.offset = [0.0, 0.0, 0.0] #SFVec3f

class ImageTexture(object):
    def __init__(self):
        self.repeatS = True #SFBool
        self.repeatT = True #SFBool
        self.url = [] #MFString

class IndexedFaceSet(object):
    def __init__(self):
        self.ccw = True #SFBool
        self.color = None #SFNode
        self.colorIndex = [] #MFInt32
        self.colorPerVertex = True #SFBool
        self.convex = True #SFBool
        self.coord = None #SFNode
        self.coordIndex = [] #MFInt32
        self.creaseAngle = 0.0 #SFFloat
        self.normal = None #SFNode
        self.normalIndex = [] #MFInt32
        self.normalPerVertex = True #SFBool
        self.solid = True #SFBool
        self.texCoord = None #SFNode
        self.texCoordIndex = [] #MFInt32

class Transform(object):
    def __init__(self):
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f
        self.center = [0.0, 0.0, 0.0] #SFVec3f
        self.rotation = [0.0, 0.0, 1.0, 0.0] #SFRotation
        self.scale = [1.0, 1.0, 1.0] #SFVec3f
        self.scaleOrientation = [0.0, 0.0, 1.0, 0.0] #SFRotation
        self.translation = [0.0, 0.0, 0.0] #SFVec3f

class TextureTransform(object):
    def __init__(self):
        self.center = [0.0, 0.0] #SFVec2f
        self.rotation = 0.0 #SFFloat
        self.scale = [1.0, 1.0] #SFVec2f
        self.translation = [0.0, 0.0] #SFVec2f

class DirectionalLight(object):
    def __init__(self):
        self.ambientIntensity = 0.0 #SFFloat
        self.color = [1.0, 1.0, 1.0] #SFColor
        self.direction = [0.0, 0.0, -1.0] #SFVec3f
        self.intensity = 1.0 #SFFloat
        self.on = True #SFBool

class PointSet(object):
    def __init__(self):
        self.color = None #SFNode
        self.coord = None #SFNode

class CoordinateInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFVec3f

class ElevationGrid(object):
    def __init__(self):
        self.ccw = True #SFBool
        self.color = None #SFNode
        self.colorPerVertex = True #SFBool
        self.creaseAngle = 0.0 #SFFloat
        self.height = [] #MFFloat
        self.normal = None #SFNode
        self.normalPerVertex = True #SFBool
        self.solid = True #SFBool
        self.texCoord = None #SFNode
        self.xDimension = 0 #SFInt32
        self.xSpacing = 0.0 #SFFloat
        self.zDimension = 0 #SFInt32
        self.zSpacing = 0.0 #SFFloat

class TextureCoordinate(object):
    def __init__(self):
        self.point = [] #MFVec2f

class MovieTexture(object):
    def __init__(self):
        self.loop = False #SFBool
        self.repeatS = True #SFBool
        self.repeatT = True #SFBool
        self.speed = 1.0 #SFFloat
        self.startTime = 0.0 #SFTime
        self.stopTime = 0.0 #SFTime
        self.url = [] #MFString

class Sound(object):
    def __init__(self):
        self.direction = [0.0, 0.0, 1.0] #SFVec3f
        self.intensity = 1.0 #SFFloat
        self.location = [0.0, 0.0, 0.0] #SFVec3f
        self.maxBack = 10.0 #SFFloat
        self.maxFront = 10.0 #SFFloat
        self.minBack = 1.0 #SFFloat
        self.minFront = 1.0 #SFFloat
        self.priority = 0.0 #SFFloat
        self.source = None #SFNode
        self.spatialize = True #SFBool

class ScalarInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFFloat

class OrientationInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFRotation

class ColorInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFColor

class WorldInfo(object):
    def __init__(self):
        self.info = [] #MFString
        self.title = '' #SFString

class Viewpoint(object):
    def __init__(self):
        self.description = '' #SFString
        self.fieldOfView = 0.78539800000000004 #SFFloat
        self.jump = True #SFBool
        self.orientation = [0.0, 0.0, 1.0, 0.0] #SFRotation
        self.position = [0.0, 0.0, 10.0] #SFVec3f

class Normal(object):
    def __init__(self):
        self.vector = [] #MFVec3f

class Material(object):
    def __init__(self):
        self.ambientIntensity = 0.20000000000000001 #SFFloat
        self.diffuseColor = [0.80000000000000004, 0.80000000000000004, 0.80000000000000004] #SFColor
        self.emissiveColor = [0.0, 0.0, 0.0] #SFColor
        self.shininess = 0.20000000000000001 #SFFloat
        self.specularColor = [0.0, 0.0, 0.0] #SFColor
        self.transparency = 0.0 #SFFloat

class PositionInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFVec3f

class Background(object):
    def __init__(self):
        self.backUrl = [] #MFString
        self.bottomUrl = [] #MFString
        self.frontUrl = [] #MFString
        self.groundAngle = [] #MFFloat
        self.groundColor = [] #MFColor
        self.leftUrl = [] #MFString
        self.rightUrl = [] #MFString
        self.skyAngle = [] #MFFloat
        self.skyColor = [0.0, 0.0, 0.0] #MFColor
        self.topUrl = [] #MFString

class NormalInterpolator(object):
    def __init__(self):
        self.key = [] #MFFloat
        self.keyValue = [] #MFVec3f

class Script(object):
    def __init__(self):
        self.directOutput = False #SFBool
        self.mustEvaluate = False #SFBool
        self.url = [] #MFString

class Box(object):
    def __init__(self):
        self.size = [2.0, 2.0, 2.0] #SFVec3f

class Extrusion(object):
    def __init__(self):
        self.beginCap = True #SFBool
        self.ccw = True #SFBool
        self.convex = True #SFBool
        self.creaseAngle = 0.0 #SFFloat
        self.crossSection = [1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0] #MFVec2f
        self.endCap = True #SFBool
        self.orientation = [0.0, 0.0, 1.0, 0.0] #MFRotation
        self.scale = [1.0, 1.0] #MFVec2f
        self.solid = True #SFBool
        self.spine = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0] #MFVec3f

class TimeSensor(object):
    def __init__(self):
        self.cycleInterval = 1.0 #SFTime
        self.enabled = True #SFBool
        self.loop = False #SFBool
        self.startTime = 0.0 #SFTime
        self.stopTime = 0.0 #SFTime

class Appearance(object):
    def __init__(self):
        self.material = None #SFNode
        self.texture = None #SFNode
        self.textureTransform = None #SFNode

class PixelTexture(object):
    def __init__(self):
        self.image = [0.0, 0.0, 0.0] #SFImage
        self.repeatS = True #SFBool
        self.repeatT = True #SFBool

class TouchSensor(object):
    def __init__(self):
        self.enabled = True #SFBool

class Switch(object):
    def __init__(self):
        self.choice = [] #MFNode
        self.whichChoice = -1 #SFInt32

class Sphere(object):
    def __init__(self):
        self.radius = 1.0 #SFFloat

class PointLight(object):
    def __init__(self):
        self.ambientIntensity = 0.0 #SFFloat
        self.attenuation = [1.0, 0.0, 0.0] #SFVec3f
        self.color = [1.0, 1.0, 1.0] #SFColor
        self.intensity = 1.0 #SFFloat
        self.location = [0.0, 0.0, 0.0] #SFVec3f
        self.on = True #SFBool
        self.radius = 100.0 #SFFloat

class ProximitySensor(object):
    def __init__(self):
        self.center = [0.0, 0.0, 0.0] #SFVec3f
        self.enabled = True #SFBool
        self.size = [0.0, 0.0, 0.0] #SFVec3f

class Coordinate(object):
    def __init__(self):
        self.point = [] #MFVec3f

class Inline(object):
    def __init__(self):
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f
        self.url = [] #MFString

class Anchor(object):
    def __init__(self):
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f
        self.description = '' #SFString
        self.parameter = [] #MFString
        self.url = [] #MFString

class Cylinder(object):
    def __init__(self):
        self.bottom = True #SFBool
        self.height = 2.0 #SFFloat
        self.radius = 1.0 #SFFloat
        self.side = True #SFBool
        self.top = True #SFBool

class VisibilitySensor(object):
    def __init__(self):
        self.center = [0.0, 0.0, 0.0] #SFVec3f
        self.enabled = True #SFBool
        self.size = [0.0, 0.0, 0.0] #SFVec3f

class Color(object):
    def __init__(self):
        self.color = [] #MFColor

class IndexedLineSet(object):
    def __init__(self):
        self.color = None #SFNode
        self.colorIndex = [] #MFInt32
        self.colorPerVertex = True #SFBool
        self.coord = None #SFNode
        self.coordIndex = [] #MFInt32

class AudioClip(object):
    def __init__(self):
        self.description = '' #SFString
        self.loop = False #SFBool
        self.pitch = 1.0 #SFFloat
        self.startTime = 0.0 #SFTime
        self.stopTime = 0.0 #SFTime
        self.url = [] #MFString

class Collision(object):
    def __init__(self):
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f
        self.collide = True #SFBool
        self.proxy = None #SFNode

class Shape(object):
    def __init__(self):
        self.appearance = None #SFNode
        self.geometry = None #SFNode

class Fog(object):
    def __init__(self):
        self.color = [1.0, 1.0, 1.0] #SFColor
        self.fogType = 'LINEAR' #SFString
        self.visibilityRange = 0.0 #SFFloat

class Cone(object):
    def __init__(self):
        self.bottom = True #SFBool
        self.bottomRadius = 1.0 #SFFloat
        self.height = 2.0 #SFFloat
        self.side = True #SFBool

class CylinderSensor(object):
    def __init__(self):
        self.autoOffset = True #SFBool
        self.diskAngle = 0.26200000000000001 #SFFloat
        self.enabled = True #SFBool
        self.maxAngle = -1.0 #SFFloat
        self.minAngle = 0.0 #SFFloat
        self.offset = 0.0 #SFFloat

class Text(object):
    def __init__(self):
        self.fontStyle = None #SFNode
        self.length = [] #MFFloat
        self.maxExtent = 0.0 #SFFloat
        self.string = [] #MFString

class Billboard(object):
    def __init__(self):
        self.axisOfRotation = [0.0, 1.0, 0.0] #SFVec3f
        self.bboxCenter = [0.0, 0.0, 0.0] #SFVec3f
        self.bboxSize = [-1.0, -1.0, -1.0] #SFVec3f

class SpotLight(object):
    def __init__(self):
        self.ambientIntensity = 0.0 #SFFloat
        self.attenuation = [1.0, 0.0, 0.0] #SFVec3f
        self.beamWidth = 1.5707960000000001 #SFFloat
        self.color = [1.0, 1.0, 1.0] #SFColor
        self.cutOffAngle = 0.78539800000000004 #SFFloat
        self.direction = [0.0, 0.0, -1.0] #SFVec3f
        self.intensity = 1.0 #SFFloat
        self.location = [0.0, 0.0, 0.0] #SFVec3f
        self.on = True #SFBool
        self.radius = 100.0 #SFFloat

