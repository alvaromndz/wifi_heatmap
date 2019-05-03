import json

class WifiMap(object):
    def __init__(self, wmap_filename):
        with open(wmap_filename, 'r') as f:
            self.wmap = json.load(f)

