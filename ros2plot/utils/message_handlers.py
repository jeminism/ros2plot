

from .message_handler_definitions import default, DATA_HANDLERS

DEFAULT_HANDLER = default

def get_message_handler(t: type):
	if t in DATA_HANDLERS:
		return DATA_HANDLERS[t]
	return DEFAULT_HANDLER

# returns the processor function to call in the callback
def get_message_processor(t: type):
    return get_message_handler(t).processor

# return the 3d plotdata if any, otherwise init_3d_plotdata expected to return None
# def get_message_3d_plotdata(t: type):
#     return get_message_handler(t).plotdata_3d()
    
def is_3d_plottable(t: type):
    return get_message_handler(t).is_3d_plottable()

# return the list of 2d fields
def get_message_2d_plottable_fields(t: type):
    return get_message_handler(t).plottable_fields_2d(t())
