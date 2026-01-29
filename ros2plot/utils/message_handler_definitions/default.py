
#processor fn is expected to take input of the raw msg, and output a dictionary of fields : data to be plotted.
def processor(msg) -> dict:
    res = {}
    introspect(msg, "", res)
    return res

def plotdata_3d():
    return None

def plottable_fields_2d(msg):
    res = {}
    introspect(msg, "", res, no_data=True)
    if len(res.items()) == 0:
        return None
    return res.keys()


NUMERIC_TYPES = (int, float, bool)

#recursive fn to introspect any msg and store its terminal fields as a dict
def introspect(msg, path, result_dict, no_data: bool=False):
    try:
        fft = msg.get_fields_and_field_types() #use this to implicitly determine if msg is a ROS msg instead of a field.
        for field in fft:
            child_path = path + "/" + field
            # print(child_path)
            if any((x+"/" in child_path or x == child_path) for x in IGNORE_FIELDS):
                continue
            self.introspect(getattr(msg, field), child_path, result_dict, no_data)
    except AttributeError:
        # NOT A ROS MSG 
        # is terminal branch
        # WARNING: type checking here MAY be limiting since this immediately removes all other data. 
        # The intended workflow would be to extend to 'plottable' types and add generic support for those data types for plotting.
        if isinstance(msg, NUMERIC_TYPES):
            result_dict[path] = None if no_data else msg
