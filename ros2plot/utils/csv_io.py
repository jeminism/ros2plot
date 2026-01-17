

import csv
from datetime import datetime


def write_to_csv(filename, data):
    with open(filename, mode="a") as f:
        writer = csv.writer(f)
        writer.writerow(data)
    
def read_from_csv(filename):
    with open(filename, mode="r") as csvfile:
        reader = csv.DictReader(csvfile)
        return list(reader)



