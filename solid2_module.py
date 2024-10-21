#
# This project uses SolidPython2 for putting together the openscad
# code.
#
# SolidPython2 doesn't have a module construct. This is my implementation of
# "module"
#
# Reference:
# https://github.com/SolidCode/SolidPython/issues/197#issuecomment-2424904674
#
# Note that this is a tweak of
# https://github.com/jeff-dh/SolidPython/tree/exportReturnValueAsModule
#

from solid2.extensions.greedy_scad_interface import *
from solid2.core.utils import indent
from solid2.core.extension_manager import default_extension_manager
import inspect
from solid2 import *

registeredModules = {}

def getCompileFunctionsHeader():
    s = ""
    for f in registeredModules:
        s += registeredModules[f]

    return s

def module(module_name, value, comment=None):
    """ Helps generate modular output """
    if not inspect.isfunction(value) and not hasattr(value, '_render'):
        raise TypeError('must be function, or a scad primitive')

    def parametersToStr(args):
        s = ""
        for a in args:
            s += str(a) + ","
        if len(s):
            #cut of trailing ","
            s = s[:-1]
        return s
    if not value in registeredModules:
        moduleCode = f"/*\n{comment}\n*/" if comment else ""
        if inspect.isfunction(value):
            argSpecs = inspect.getfullargspec(value).args
            parameters = [ScadValue(p) for p in argSpecs]
            moduleCode += f"module {module_name}({parametersToStr(argSpecs)}){{\n"
            moduleCode += indent(value(*parameters)._render())
        else:
            moduleCode = f"module {module_name}(){{\n"
            moduleCode += indent(value()._render())
        moduleCode += "}\n"
        registeredModules[value] = moduleCode

    # fake union to enable mixins.
    return lambda *args : union() + ScadValue(f"{module_name}({parametersToStr(args)});\n")

def exportReturnValueAsModule(func):
    """ Use this as a decorator for functions. """
    return module(func.__name__, func)

# Pre render ensures that the module defs are output before the
# scad tree
default_extension_manager.register_pre_render(lambda root : getCompileFunctionsHeader())

