
from .handler_db import introspect_data, introspect_types

NUMERIC_TYPES = (int, float, bool)

#processor fn is expected to take input of the raw msg, and output a dictionary of fields : data to be plotted.
def processor(msg) -> dict:
    res = {f: t for f, t in introspect_data(msg).items() if isinstance(t, NUMERIC_TYPES)}
    return res


def plottable_fields_2d(msg):
    res = [f for f, t in introspect_types(msg).items() if t in NUMERIC_TYPES]
    return res

