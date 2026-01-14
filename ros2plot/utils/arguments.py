
import argparse

TOPIC_NAME='topic_name'
TOPIC_TYPE='topic_type'
FIELDS='fields'
X_FIELD='x_field'
CSV='csv'
LOG_STATS='log_stats'

#just to quiet the output on an error, otherwise argparse prints directly to stderr which will mess up the asciimatics output
class SilentParser(argparse.ArgumentParser):
    def error(self, message):
        raise ValueError(message)

    def exit(self, status=0, message=None):
        if message:
            raise ValueError(message)

def to_optional(s):
    return '--'+s.replace('_','-')

def set_args(parser):
    parser.add_argument(TOPIC_NAME, nargs="?", default=None, help='Name of the topic to subscribe')
    parser.add_argument(TOPIC_TYPE, nargs="?", default=None, help='Type of topic to subscribe to. If missing, will internally attempt to automatically determine the topic type.')
    parser.add_argument(to_optional(FIELDS), nargs='*', help='Specific fields to plot. Expects directory style path.')
    parser.add_argument(to_optional(X_FIELD), nargs=1, help='Specific field to use as x axis. Expects directory style path. If missing, will default to system time')
    parser.add_argument(to_optional(CSV), nargs=1, help='CSV file to pull values from')
    parser.add_argument(to_optional(LOG_STATS), action="store_true", help='Enable stat logging')

# do input processing and return to caller
def get_args(inputs, silent=False):
    parser = SilentParser() if silent else argparse.ArgumentParser()
    set_args(parser)
    try:
        raw = vars(parser.parse_args(inputs))
        # just strip all leading slashes for field names as a rule
        res = {
            TOPIC_NAME: raw[TOPIC_NAME].lstrip("/") if raw[TOPIC_NAME] != None else None,
            TOPIC_TYPE: raw[TOPIC_TYPE],
            FIELDS: [f.lstrip("/") for f in raw[FIELDS]] if raw[FIELDS]!=None else None,
            X_FIELD: raw[X_FIELD][0].lstrip("/")if raw[X_FIELD]!=None else None,
            CSV: raw[CSV][0] if raw[CSV]!=None else None,
            LOG_STATS: raw[LOG_STATS]
        }
        return res
    except Exception as e:
        raise Exception()
    