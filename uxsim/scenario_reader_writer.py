"""
Scenario reader and writer using toml.
"""

import functools
import inspect
from collections import defaultdict as ddict
import tomlkit
import numpy as np

def get_function_args_list(func):
    signature = inspect.signature(func)
    args = {
        name: param.default
        if param.default != param.empty else "nodefaultvalue_get_function_args_list"
        for name, param in signature.parameters.items()
    }
    for key in list(args):
        if key == "s":
            del args[key]
    return args

def extract_network_attributes(instance, attributes):
    attrs = {}
    for attr in attributes:
        try:
            attrs[attr] = getattr(instance, attr)
        except AttributeError:
            print(instance, attr)
            pass
        
    for key in list(attrs):
        if attrs[key] == attributes[key]:
            del attrs[key]
            continue
        if str(type(attrs[key])) == "<class 'uxsim.uxsim.World'>":
            del attrs[key]
            continue
        if str(type(attrs[key])) == "<class 'uxsim.uxsim.Node'>":
            attrs[key] = attrs[key].name
        if str(type(attrs[key])) == "<class 'uxsim.uxsim.Link'>":
            attrs[key] = attrs[key].name
        # if str(type(attrs[key])) == str(World):
        #     del attrs[key]
        #     continue
        # if str(type(attrs[key])) == str(Node):
        #     attrs[key] = attrs[key].name
        # if str(type(attrs[key])) == str(Link):
        #     attrs[key] = attrs[key].name
    return attrs


def convert_tomlkit_type(value):
    #print(value, type(value))
    if isinstance(value, (tomlkit.items.Integer, tomlkit.items.Float, tomlkit.items.String, tomlkit.items.Bool)):
        return value.unwrap()
    elif isinstance(value, tomlkit.items.AoT):
        return [convert_tomlkit_type(item) for item in value]
    elif isinstance(value, tomlkit.items.Array):
        return [convert_tomlkit_type(item) for item in value]
    elif isinstance(value, tomlkit.items.Table):
        return convert_tomlkit_dict(value)
    else:
        return value

def convert_tomlkit_dict(data):
    return {k: convert_tomlkit_type(v) for k, v in data.items()}

def extract_network_info(W):
    """
    Extract network info (Node and Link) as toml-compatible dict.

    Parameters
    ----------
    W : World
        The world object to save the network from.
    """
    NODE_ATTRS = get_function_args_list(W.NODES[0].__init__)
    LINK_ATTRS = get_function_args_list(W.LINKS[0].__init__)
    # NODE_ATTRS = get_function_args_list(Node.__init__)
    # LINK_ATTRS = get_function_args_list(Link.__init__)
    network = {}

    if W.meta_data != {}:
        network["World"] = {
            "meta_data": {str(key):W.meta_data[key] for key in W.meta_data}
        }

    network["Node"] = [extract_network_attributes(node, NODE_ATTRS) for node in W.NODES]

    network["Link"] = [extract_network_attributes(link, LINK_ATTRS) for link in W.LINKS]

    return network
    
def convert_numpy_types(obj):
    #TODO: numpy data types are not fully supported
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, tuple):
        return tuple(convert_numpy_types(item) for item in obj)
    return obj

def save_network_as_toml(W, fname):
    """
    Save network (Node and Link) to a toml file.

    Parameters
    ----------
    W : World
        The world object to save the network from.
    fname : str
        The file name of the toml file to save.
    """
    network = extract_network_info(W)

    network = {k: convert_numpy_types(v) for k, v in network.items()}
    
    with open(fname, 'w', encoding='utf-8') as file:
       tomlkit.dump(network, file)

def load_network_from_toml(W, fname):
    """
    Load a network (Node and Link) from a toml file.

    Parameters
    ----------
    W : World
        The world object to load the network into.
    fname : str
        The file name of the toml file to load.
    """
    print(f"loading network from file `{fname}`")
    with open(fname, 'r', encoding='utf-8') as file:
        network = convert_tomlkit_dict(tomlkit.load(file))
    
    if "World" in network and "meta_data" in network["World"]:
        print(" Scenario meta data:")
        for key in network["World"]["meta_data"]:
            print(f'  {key}: {network["World"]["meta_data"][key]}')
        

    for node in network.get('Node', []):
        n = W.addNode(**node)

    for link in network.get('Link', []):
        l = W.addLink(**link)


def demand_info_record(func):
    """
    A decorator to record arguments of World.addVehicle, World.adddemand, etc used in secenario definition in World object.
    """

    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        # 関数名を取得
        func_name = func.__name__
        
        # 引数の情報を取得
        bound_args = inspect.signature(func).bind(self, *args, **kwargs)
        bound_args.apply_defaults()
        
        # self引数を除外
        arg_dict = dict(list(bound_args.arguments.items())[1:])
        
        # direct_call=Trueが含まれている場合のみ保存
        if arg_dict.get('direct_call') == True:
            # クラスにdict_func_args属性がなければ作成
            if not hasattr(self.__class__, 'demand_info'):
                self.__class__.demand_info = ddict(list)
            
            # direct_callを除いた引数を保存
            arg_dict_to_save = {k: v for k, v in arg_dict.items() if k != 'direct_call'}
            #arg_dict_to_save["] = func_name
            
            # 関数名と引数をdict_func_argsに格納
            self.__class__.demand_info[func_name].append(arg_dict_to_save)
        
        # 元の関数を呼び出す
        return func(self, *args, **kwargs)
    
    return wrapper

def extract_demand_info(W):
    demand_info = dict(W.demand_info)
    for demand_type in demand_info.values():
        for demand in demand_type:
            for key in list(demand.keys()):
                if str(type(demand[key])) == "<class 'uxsim.uxsim.Node'>":
                    demand[key] = demand[key].name
                if str(type(demand[key])) == "<class 'uxsim.uxsim.Link'>":
                    demand[key] = demand[key].name
                if demand[key] ==  None:
                    del demand[key]
    
    # toml_string = tomlkit.dumps(demand_info)
    # print(toml_string)

    return demand_info

def save_demand_as_toml(W, fname):
    """
    Save demand (addVehcile, adddemand, etc) to a toml file.

    Parameters
    ----------
    W : World
        The world object to save the network from.
    fname : str
        The file name of the toml file to save.
    """
    demand = extract_demand_info(W)

    demand = {k: convert_numpy_types(v) for k, v in demand.items()}
    
    with open(fname, 'w', encoding='utf-8') as file:
       tomlkit.dump(demand, file)
    

def load_demand_from_toml(W, fname):
    print(f"loading demand from file `{fname}`")

    with open(fname, 'r', encoding='utf-8') as file:
        demand = convert_tomlkit_dict(tomlkit.load(file))

    for dem in demand.get("addVehicle", []):
        W.addVehicle(**dem)
    for dem in demand.get("adddemand", []):
        W.adddemand(**dem)
    for dem in demand.get("adddemand_point2point", []):
        W.adddemand_point2point(**dem)
    for dem in demand.get("adddemand_area2area", []):
        W.adddemand_area2area(**dem)


def save_scenario_as_toml(W, fname):
    """
    Save entire siulation scenario (network and demand) to a toml file.

    Parameters
    ----------
    W : World
        The world object to save the network from.
    fname : str
        The file name of the toml file to save.
    """
    network = extract_network_info(W)
    demand = extract_demand_info(W)
    scenario = network | demand

    with open(fname, 'w', encoding='utf-8') as file:
        tomlkit.dump(scenario, file)

def load_scenario_from_toml(W, fname):
    load_network_from_toml(W, fname)
    load_demand_from_toml(W, fname)

