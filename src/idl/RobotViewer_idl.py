# Python stubs generated by omniidl from RobotViewer.idl

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA

_omnipy.checkVersion(3,0, __file__)


#
# Start of module "robotviewer_corba"
#
__name__ = "robotviewer_corba"
_0_robotviewer_corba = omniORB.openModule("robotviewer_corba", r"RobotViewer.idl")
_0_robotviewer_corba__POA = omniORB.openModule("robotviewer_corba__POA", r"RobotViewer.idl")


# typedef ... DoubleSeq
class DoubleSeq:
    _NP_RepositoryId = "IDL:robotviewer_corba/DoubleSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_robotviewer_corba.DoubleSeq = DoubleSeq
_0_robotviewer_corba._d_DoubleSeq  = (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_double, 0)
_0_robotviewer_corba._ad_DoubleSeq = (omniORB.tcInternal.tv_alias, DoubleSeq._NP_RepositoryId, "DoubleSeq", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_double, 0))
_0_robotviewer_corba._tc_DoubleSeq = omniORB.tcInternal.createTypeCode(_0_robotviewer_corba._ad_DoubleSeq)
omniORB.registerType(DoubleSeq._NP_RepositoryId, _0_robotviewer_corba._ad_DoubleSeq, _0_robotviewer_corba._tc_DoubleSeq)
del DoubleSeq

# typedef ... ElementList
class ElementList:
    _NP_RepositoryId = "IDL:robotviewer_corba/ElementList:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_robotviewer_corba.ElementList = ElementList
_0_robotviewer_corba._d_ElementList  = (omniORB.tcInternal.tv_sequence, (omniORB.tcInternal.tv_string,0), 0)
_0_robotviewer_corba._ad_ElementList = (omniORB.tcInternal.tv_alias, ElementList._NP_RepositoryId, "ElementList", (omniORB.tcInternal.tv_sequence, (omniORB.tcInternal.tv_string,0), 0))
_0_robotviewer_corba._tc_ElementList = omniORB.tcInternal.createTypeCode(_0_robotviewer_corba._ad_ElementList)
omniORB.registerType(ElementList._NP_RepositoryId, _0_robotviewer_corba._ad_ElementList, _0_robotviewer_corba._tc_ElementList)
del ElementList

# typedef ... ShortSeq
class ShortSeq:
    _NP_RepositoryId = "IDL:robotviewer_corba/ShortSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_robotviewer_corba.ShortSeq = ShortSeq
_0_robotviewer_corba._d_ShortSeq  = (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_short, 0)
_0_robotviewer_corba._ad_ShortSeq = (omniORB.tcInternal.tv_alias, ShortSeq._NP_RepositoryId, "ShortSeq", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_short, 0))
_0_robotviewer_corba._tc_ShortSeq = omniORB.tcInternal.createTypeCode(_0_robotviewer_corba._ad_ShortSeq)
omniORB.registerType(ShortSeq._NP_RepositoryId, _0_robotviewer_corba._ad_ShortSeq, _0_robotviewer_corba._tc_ShortSeq)
del ShortSeq

# typedef ... HomogeneousMatrix
class HomogeneousMatrix:
    _NP_RepositoryId = "IDL:robotviewer_corba/HomogeneousMatrix:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_robotviewer_corba.HomogeneousMatrix = HomogeneousMatrix
_0_robotviewer_corba._d_HomogeneousMatrix  = (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], 0)
_0_robotviewer_corba._ad_HomogeneousMatrix = (omniORB.tcInternal.tv_alias, HomogeneousMatrix._NP_RepositoryId, "HomogeneousMatrix", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], 0))
_0_robotviewer_corba._tc_HomogeneousMatrix = omniORB.tcInternal.createTypeCode(_0_robotviewer_corba._ad_HomogeneousMatrix)
omniORB.registerType(HomogeneousMatrix._NP_RepositoryId, _0_robotviewer_corba._ad_HomogeneousMatrix, _0_robotviewer_corba._tc_HomogeneousMatrix)
del HomogeneousMatrix

# interface RobotViewer
_0_robotviewer_corba._d_RobotViewer = (omniORB.tcInternal.tv_objref, "IDL:robotviewer_corba/RobotViewer:1.0", "RobotViewer")
omniORB.typeMapping["IDL:robotviewer_corba/RobotViewer:1.0"] = _0_robotviewer_corba._d_RobotViewer
_0_robotviewer_corba.RobotViewer = omniORB.newEmptyClass()
class RobotViewer :
    _NP_RepositoryId = _0_robotviewer_corba._d_RobotViewer[1]

    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")

    _nil = CORBA.Object._nil

    
    # exception InvalidKey
    _0_robotviewer_corba.RobotViewer.InvalidKey = omniORB.newEmptyClass()
    class InvalidKey (CORBA.UserException):
        _NP_RepositoryId = "IDL:robotviewer_corba/RobotViewer/InvalidKey:1.0"

        _NP_ClassName = "robotviewer_corba.RobotViewer.InvalidKey"

        def __init__(self, reason):
            CORBA.UserException.__init__(self, reason)
            self.reason = reason
    
    _d_InvalidKey  = (omniORB.tcInternal.tv_except, InvalidKey, InvalidKey._NP_RepositoryId, "InvalidKey", "reason", (omniORB.tcInternal.tv_string,0))
    _tc_InvalidKey = omniORB.tcInternal.createTypeCode(_d_InvalidKey)
    omniORB.registerType(InvalidKey._NP_RepositoryId, _d_InvalidKey, _tc_InvalidKey)


_0_robotviewer_corba.RobotViewer = RobotViewer
_0_robotviewer_corba._tc_RobotViewer = omniORB.tcInternal.createTypeCode(_0_robotviewer_corba._d_RobotViewer)
omniORB.registerType(RobotViewer._NP_RepositoryId, _0_robotviewer_corba._d_RobotViewer, _0_robotviewer_corba._tc_RobotViewer)

# RobotViewer operations and attributes
RobotViewer._d_createElement = (((omniORB.tcInternal.tv_string,0), (omniORB.tcInternal.tv_string,0), (omniORB.tcInternal.tv_string,0)), (omniORB.tcInternal.tv_boolean, ), None)
RobotViewer._d_setRobotJointRank = (((omniORB.tcInternal.tv_string,0), (omniORB.tcInternal.tv_string,0)), (omniORB.tcInternal.tv_boolean, ), None)
RobotViewer._d_destroyElement = (((omniORB.tcInternal.tv_string,0), ), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_enableElement = (((omniORB.tcInternal.tv_string,0), ), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_disableElement = (((omniORB.tcInternal.tv_string,0), ), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_setTransparency = (((omniORB.tcInternal.tv_string,0), omniORB.tcInternal.tv_double), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_updateElementConfig = (((omniORB.tcInternal.tv_string,0), omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"]), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_updateElementConfig2 = (((omniORB.tcInternal.tv_string,0), omniORB.typeMapping["IDL:robotviewer_corba/HomogeneousMatrix:1.0"], omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"]), (omniORB.tcInternal.tv_boolean, ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_getElementConfig = (((omniORB.tcInternal.tv_string,0), ), (omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_getElementConfig2 = (((omniORB.tcInternal.tv_string,0), ), (omniORB.typeMapping["IDL:robotviewer_corba/HomogeneousMatrix:1.0"], omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"]), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_test = ((), (omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], ), None)
RobotViewer._d_listElements = ((), (omniORB.typeMapping["IDL:robotviewer_corba/ElementList:1.0"], ), None)
RobotViewer._d_listElementDofs = (((omniORB.tcInternal.tv_string,0), ), (omniORB.typeMapping["IDL:robotviewer_corba/ElementList:1.0"], ), None)
RobotViewer._d_listCameras = ((), (omniORB.typeMapping["IDL:robotviewer_corba/ElementList:1.0"], ), None)
RobotViewer._d_getCameraConfig = (((omniORB.tcInternal.tv_string,0), ), (omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_getCameraInfo = (((omniORB.tcInternal.tv_string,0), ), ((omniORB.tcInternal.tv_string,0), ), None)
RobotViewer._d_setCameraOpenCVParams = (((omniORB.tcInternal.tv_string,0), omniORB.tcInternal.tv_short, omniORB.tcInternal.tv_short, omniORB.tcInternal.tv_double, omniORB.tcInternal.tv_double, omniORB.tcInternal.tv_double, omniORB.tcInternal.tv_double), (), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_listMeshes = ((), (omniORB.typeMapping["IDL:robotviewer_corba/ShortSeq:1.0"], ), None)
RobotViewer._d_getGlConfig = ((omniORB.tcInternal.tv_short, ), (omniORB.typeMapping["IDL:robotviewer_corba/DoubleSeq:1.0"], ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_printMesh = ((omniORB.tcInternal.tv_short, ), ((omniORB.tcInternal.tv_string,0), ), {_0_robotviewer_corba.RobotViewer.InvalidKey._NP_RepositoryId: _0_robotviewer_corba.RobotViewer._d_InvalidKey})
RobotViewer._d_Ping = ((), ((omniORB.tcInternal.tv_string,0), ), None)
RobotViewer._d_start_record = ((), (), None)
RobotViewer._d_stop_record = ((), (), None)

# RobotViewer object reference
class _objref_RobotViewer (CORBA.Object):
    _NP_RepositoryId = RobotViewer._NP_RepositoryId

    def __init__(self):
        CORBA.Object.__init__(self)

    def createElement(self, *args):
        return _omnipy.invoke(self, "createElement", _0_robotviewer_corba.RobotViewer._d_createElement, args)

    def setRobotJointRank(self, *args):
        return _omnipy.invoke(self, "setRobotJointRank", _0_robotviewer_corba.RobotViewer._d_setRobotJointRank, args)

    def destroyElement(self, *args):
        return _omnipy.invoke(self, "destroyElement", _0_robotviewer_corba.RobotViewer._d_destroyElement, args)

    def enableElement(self, *args):
        return _omnipy.invoke(self, "enableElement", _0_robotviewer_corba.RobotViewer._d_enableElement, args)

    def disableElement(self, *args):
        return _omnipy.invoke(self, "disableElement", _0_robotviewer_corba.RobotViewer._d_disableElement, args)

    def setTransparency(self, *args):
        return _omnipy.invoke(self, "setTransparency", _0_robotviewer_corba.RobotViewer._d_setTransparency, args)

    def updateElementConfig(self, *args):
        return _omnipy.invoke(self, "updateElementConfig", _0_robotviewer_corba.RobotViewer._d_updateElementConfig, args)

    def updateElementConfig2(self, *args):
        return _omnipy.invoke(self, "updateElementConfig2", _0_robotviewer_corba.RobotViewer._d_updateElementConfig2, args)

    def getElementConfig(self, *args):
        return _omnipy.invoke(self, "getElementConfig", _0_robotviewer_corba.RobotViewer._d_getElementConfig, args)

    def getElementConfig2(self, *args):
        return _omnipy.invoke(self, "getElementConfig2", _0_robotviewer_corba.RobotViewer._d_getElementConfig2, args)

    def test(self, *args):
        return _omnipy.invoke(self, "test", _0_robotviewer_corba.RobotViewer._d_test, args)

    def listElements(self, *args):
        return _omnipy.invoke(self, "listElements", _0_robotviewer_corba.RobotViewer._d_listElements, args)

    def listElementDofs(self, *args):
        return _omnipy.invoke(self, "listElementDofs", _0_robotviewer_corba.RobotViewer._d_listElementDofs, args)

    def listCameras(self, *args):
        return _omnipy.invoke(self, "listCameras", _0_robotviewer_corba.RobotViewer._d_listCameras, args)

    def getCameraConfig(self, *args):
        return _omnipy.invoke(self, "getCameraConfig", _0_robotviewer_corba.RobotViewer._d_getCameraConfig, args)

    def getCameraInfo(self, *args):
        return _omnipy.invoke(self, "getCameraInfo", _0_robotviewer_corba.RobotViewer._d_getCameraInfo, args)

    def setCameraOpenCVParams(self, *args):
        return _omnipy.invoke(self, "setCameraOpenCVParams", _0_robotviewer_corba.RobotViewer._d_setCameraOpenCVParams, args)

    def listMeshes(self, *args):
        return _omnipy.invoke(self, "listMeshes", _0_robotviewer_corba.RobotViewer._d_listMeshes, args)

    def getGlConfig(self, *args):
        return _omnipy.invoke(self, "getGlConfig", _0_robotviewer_corba.RobotViewer._d_getGlConfig, args)

    def printMesh(self, *args):
        return _omnipy.invoke(self, "printMesh", _0_robotviewer_corba.RobotViewer._d_printMesh, args)

    def Ping(self, *args):
        return _omnipy.invoke(self, "Ping", _0_robotviewer_corba.RobotViewer._d_Ping, args)

    def start_record(self, *args):
        return _omnipy.invoke(self, "start_record", _0_robotviewer_corba.RobotViewer._d_start_record, args)

    def stop_record(self, *args):
        return _omnipy.invoke(self, "stop_record", _0_robotviewer_corba.RobotViewer._d_stop_record, args)

    __methods__ = ["createElement", "setRobotJointRank", "destroyElement", "enableElement", "disableElement", "setTransparency", "updateElementConfig", "updateElementConfig2", "getElementConfig", "getElementConfig2", "test", "listElements", "listElementDofs", "listCameras", "getCameraConfig", "getCameraInfo", "setCameraOpenCVParams", "listMeshes", "getGlConfig", "printMesh", "Ping", "start_record", "stop_record"] + CORBA.Object.__methods__

omniORB.registerObjref(RobotViewer._NP_RepositoryId, _objref_RobotViewer)
_0_robotviewer_corba._objref_RobotViewer = _objref_RobotViewer
del RobotViewer, _objref_RobotViewer

# RobotViewer skeleton
__name__ = "robotviewer_corba__POA"
class RobotViewer (PortableServer.Servant):
    _NP_RepositoryId = _0_robotviewer_corba.RobotViewer._NP_RepositoryId


    _omni_op_d = {"createElement": _0_robotviewer_corba.RobotViewer._d_createElement, "setRobotJointRank": _0_robotviewer_corba.RobotViewer._d_setRobotJointRank, "destroyElement": _0_robotviewer_corba.RobotViewer._d_destroyElement, "enableElement": _0_robotviewer_corba.RobotViewer._d_enableElement, "disableElement": _0_robotviewer_corba.RobotViewer._d_disableElement, "setTransparency": _0_robotviewer_corba.RobotViewer._d_setTransparency, "updateElementConfig": _0_robotviewer_corba.RobotViewer._d_updateElementConfig, "updateElementConfig2": _0_robotviewer_corba.RobotViewer._d_updateElementConfig2, "getElementConfig": _0_robotviewer_corba.RobotViewer._d_getElementConfig, "getElementConfig2": _0_robotviewer_corba.RobotViewer._d_getElementConfig2, "test": _0_robotviewer_corba.RobotViewer._d_test, "listElements": _0_robotviewer_corba.RobotViewer._d_listElements, "listElementDofs": _0_robotviewer_corba.RobotViewer._d_listElementDofs, "listCameras": _0_robotviewer_corba.RobotViewer._d_listCameras, "getCameraConfig": _0_robotviewer_corba.RobotViewer._d_getCameraConfig, "getCameraInfo": _0_robotviewer_corba.RobotViewer._d_getCameraInfo, "setCameraOpenCVParams": _0_robotviewer_corba.RobotViewer._d_setCameraOpenCVParams, "listMeshes": _0_robotviewer_corba.RobotViewer._d_listMeshes, "getGlConfig": _0_robotviewer_corba.RobotViewer._d_getGlConfig, "printMesh": _0_robotviewer_corba.RobotViewer._d_printMesh, "Ping": _0_robotviewer_corba.RobotViewer._d_Ping, "start_record": _0_robotviewer_corba.RobotViewer._d_start_record, "stop_record": _0_robotviewer_corba.RobotViewer._d_stop_record}

RobotViewer._omni_skeleton = RobotViewer
_0_robotviewer_corba__POA.RobotViewer = RobotViewer
omniORB.registerSkeleton(RobotViewer._NP_RepositoryId, RobotViewer)
del RobotViewer
__name__ = "robotviewer_corba"

#
# End of module "robotviewer_corba"
#
__name__ = "RobotViewer_idl"

_exported_modules = ( "robotviewer_corba", )

# The end.
