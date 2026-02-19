
import sys

DATA_HANDLERS={}

REQUIRED={"processor", "plottable_fields_2d"}

def register_module(msg_type, msg_handler_module_name):
    msg_handler_module = sys.modules[msg_handler_module_name]
    missing = REQUIRED - msg_handler_module.__dict__.keys()
    if missing:
        raise TypeError(f"{msg_handler_module.__name__} missing: {missing}")

    DATA_HANDLERS[msg_type] = msg_handler_module




### HELPER FUNCTIONS

# get 1D dict of field : value pairs of the incoming message
def introspect_data(msg) -> dict:
    res = {}
    introspect(msg, "", res, False)
    return res

# get 1D dict of field : type pairs of a message
def introspect_types(msg) -> list:
    res = {}
    introspect(msg, "", res, True)
    return res

#recursive fn to introspect any msg and store its terminal fields as a flattened 1D dict
def introspect(msg, path, result_dict, no_data: bool=False):
    try:
        fft = msg.get_fields_and_field_types() #use this to implicitly determine if msg is a ROS msg instead of a field.
        for field in fft:
            child_path = path + "/" + field
            # print(child_path)
            # if any((x+"/" in child_path or x == child_path) for x in IGNORE_FIELDS):
            #     continue
            introspect(getattr(msg, field), child_path, result_dict, no_data)
    except AttributeError:
        # NOT A ROS MSG 
        # is terminal branch
        result_dict[path] = type(msg) if no_data else msg