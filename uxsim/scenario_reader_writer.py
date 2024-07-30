"""
Scenario reader and writer using toml.
"""

import functools
import inspect
from collections import defaultdict as ddict
import pickle

def demand_info_record(func):
    """
    A decorator to record arguments of `World.addVehicle`, `World.adddemand`, etc used in secenario definition in World object.
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
            
            # direct_callを除いた引数を保存
            arg_dict_to_save = {k: v for k, v in arg_dict.items() if k != 'direct_call'}
            #arg_dict_to_save["] = func_name
            
            # 関数名と引数をdict_func_argsに格納
            self.demand_info[func_name].append(arg_dict_to_save)
        
        # 元の関数を呼び出す
        return func(self, *args, **kwargs)
    
    return wrapper

def instance_to_arg_dict(instance):
    # クラスの__init__メソッドのシグネチャを取得
    signature = inspect.signature(instance.__class__.__init__)
    
    # インスタンス変数を取得
    instance_vars = vars(instance)
    
    # 引数リストを作成
    args = {}
    for param_name, param in signature.parameters.items():
        if param_name == 'self':
            continue
        
        value = instance_vars.get(param_name, param.default)
      
        if value == param.default:
            continue
        
        args[param_name] = value
        for param in list(args):
            if param == "W":
                del args[param]            
            elif str(type(args[param])) == "<class 'uxsim.uxsim.Node'>":
                args[param] = args[param].name
            elif str(type(args[param])) == "<class 'uxsim.uxsim.Link'>":
                args[param] = args[param].name

    return args

def save_scenario(W, fname, network=True, demand=True):
    out = {"meta_data": W.meta_data}

    if network:
        nodes = []
        for n in W.NODES:
            nodes.append(instance_to_arg_dict(n))
        links = []
        for l in W.LINKS:
            links.append(instance_to_arg_dict(l))
        network = {
            "Nodes": nodes,
            "Links": links
        }
        out = out | network
    if demand:
        out = out | dict(W.demand_info)
    
    # from pprint import pprint
    # pprint(out)
    with open(fname, "wb") as f:
        pickle.dump(out, f)
    

def load_scenario(W, fname, network=True, demand=True):
    with open(fname, "rb") as f:
        dat = pickle.load(f)
    
    # from pprint import pprint
    # pprint(dat)

    print(f"loading scenario from '{fname}'")
    if dat["meta_data"]:
        if type(dat["meta_data"]) is dict:
            for key in dat["meta_data"]:
                print("", key, ":", dat["meta_data"][key])
        else:
            print("", dat["meta_data"])


    if network:
        for n in dat["Nodes"]:
            W.addNode(**n)
        for l in dat["Links"]:
            W.addLink(**l)
        print(" Number of loaded nodes:", len(dat["Nodes"]))
        print(" Number of loaded links:", len(dat["Links"]))
    
    if demand:
        for demand_type in dat:
            if demand_type == 'adddemand':
                for dem in dat[demand_type]:
                    W.adddemand(**dem)
                print(f" Number of loaded `{demand_type}`s:", len( dat[demand_type]))
            elif demand_type == 'adddemand_point2point':
                for dem in dat[demand_type]:
                    W.adddemand_point2point(**dem)
                print(f" Number of loaded `{demand_type}`s:", len( dat[demand_type]))
            if demand_type == 'adddemand_area2area':
                for dem in dat[demand_type]:
                    W.adddemand_area2area(**dem)
                print(f" Number of loaded `{demand_type}`s:", len( dat[demand_type]))
        




