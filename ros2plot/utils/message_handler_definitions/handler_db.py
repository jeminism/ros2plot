import sys

FIELD_3D = "3D" #this is used because the 3d field is just the topic name, but this is not known here.

DATA_HANDLERS={}

REQUIRED={"processor", "is_3d_plottable", "plottable_fields_2d"}

def register_module(msg_type, msg_handler_module_name):
    msg_handler_module = sys.modules[msg_handler_module_name]
    missing = REQUIRED - msg_handler_module.__dict__.keys()
    if missing:
        raise TypeError(f"{msg_handler_module.__name__} missing: {missing}")

    DATA_HANDLERS[msg_type] = msg_handler_module

