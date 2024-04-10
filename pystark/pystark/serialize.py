import pystark
import numpy as np
import pickle
from typing import List, Tuple, Optional

def get_full_type(obj):
    module = obj.__class__.__module__
    if module is None or module == str.__class__.__module__:
        return obj.__class__.__name__  # Avoid reporting __builtin__
    else:
        return module + '.' + obj.__class__.__name__

def get_type_from_string(enum_string):
    components = enum_string.split('.')
    module = __import__(components[0])
    for comp in components[1:]:
        module = getattr(module, comp)
    return module

def to_python(obj):
    attrs = {}
    attrs["_type"] = get_full_type(obj)
    for attr in dir(obj):
        if attr.startswith('__') or callable(getattr(obj, attr)):
            continue  # Skip magic methods, callables and type definitions
        value = getattr(obj, attr)
        if 'pystark' in str(type(value)):
            if '@entries' in dir(value):
                value = str(value)  # This is an enum. Just save the type name
            else:
                value = to_python(value)  # Recursion
        attrs[attr] = value
    return attrs

def from_python(serialized, obj=None):
    if obj is None:
        obj = get_type_from_string(serialized["_type"])()

    for attr, value in serialized.items():
        if attr == "_type":
            continue
        elif isinstance(value, dict) and "pystark" in value["_type"]:
            value = from_python(value, getattr(obj, attr))  # Recursion
        else:
            if isinstance(value, str) and "pystark" in value:
                value = get_type_from_string(value)  # Enum
        setattr(obj, attr, value)
    return obj

def print_helloworld():
    print("Hello world!")

class SFXSIM_PrescribedPositions:
    def __init__(self):
        self.indices : Optional[np.array] = None
        self.positions : List[np.array] = []

class SFXSIM_Cloth:
    def __init__(self):
        self.name : Optional[str] = None
        self.vertices : Optional[np.array] = None
        self.triangles : Optional[np.array] = None
        self.params = []  # This switches between pystark.Surface.Params and the serialized version
        self.prescribed_positions : List[SFXSIM_PrescribedPositions] = []

class SFXSIM:
    def __init__(self):

        # General
        self.settings = None   # This switches between pystark.Settings and the serialized version
        self.contact_params = None  # This switches between pystark.EnergyFrictionalContact.GlobalParams and the serialized version
        
        # Objects
        self.cloths : List[SFXSIM_Cloth] = []

    def add_cloth(self, name : str, vertices : np.array, triangles : np.array):
        cloth = SFXSIM_Cloth()
        cloth.name = name
        cloth.vertices = vertices
        cloth.triangles = triangles
        self.cloths.append(cloth)

    def add_cloth_prescribed_positions(self, indices : np.array):
        prescribed_positions = SFXSIM_PrescribedPositions()
        prescribed_positions.indices = indices
        self.cloths[-1].prescribed_positions.append(prescribed_positions)

    def add_cloth_params_update(self, params : pystark.Surface.Params):
        self.cloths[-1].params.append(params)

    def add_cloth_prescribed_positions_update(self, positions : np.array):
        prescribed_positions = self.cloths[-1].prescribed_positions[-1]
        if len(prescribed_positions.indices) != len(positions):
            raise ValueError("The number of indices and positions must match")
        prescribed_positions.positions.append(positions)

    def is_valid(self):
        return self.settings is not None
    
    def to_pickle_form(self):
        self.settings = to_python(self.settings)
        self.contact_params = to_python(self.contact_params)
        for cloth in self.cloths:
            for param in cloth.params:
                param = to_python(param)

    def to_pystark_form(self):
        self.settings = from_python(self.settings)
        self.contact_params = from_python(self.contact_params)
        for cloth in self.cloths:
            for param in cloth.params:
                param = from_python(param)
    
    def write(self, filename):
        if not self.is_valid():
            raise ValueError("The SFXSIM object is not valid")
        self.to_pickle_form()
        pickle.dump(self, open(filename, "wb"))
        self.to_pystark_form()

