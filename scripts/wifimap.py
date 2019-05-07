import json
import heapq
import pandas as pd

def sim_cosine(z, x):
    zmag = 0
    xmag = 0
    dot = 0
    for mac, stren in z.items():
        if mac in x:
            dot += stren * x[mac]
            xmag += x[mac]**2
        zmag += stren**2
    
    if zmag == 0 or xmag == 0:
        return 0

    return dot / zmag**.5 / xmag**.5

class Similarity(object):
    def __init__(self, loc, sim):
        self.loc = loc
        self.sim = sim
    
    def __lt__(self, other):
        return self.sim < other.sim
    
    def __eq__(self, other):
        return self.sim == other.sim
    
    def __str__(self):
        return str(self.loc + (self.sim,))



class WifiMapOld(object):
    def __init__(self, wmap_filename):
        with open(wmap_filename, 'r') as f:
            raw_wmap = json.load(f)
        
        self.wmap = {}
        for z in raw_wmap:
            aps = {ap['MAC'] : ap['strength'] for ap in z['aps']}
            self.wmap[(z['x'], z['y'])] = aps
        
    def localize(self, meas):
        """ 
        returns best guess of location and error
        meas should be list of dicts 
        """
        sims = [Similarity(loc, sim_cosine(meas, aps)) for loc, aps in self.wmap.items()]
        most_sim = heapq.nlargest(10, sims)

        for sim in most_sim:
            print(sim)

class WifiMap(object):
    def __init__(self, wmap_filename):
        with open(wmap_filename, 'r') as f:
            raw_wmap = json.load(f)
        
        macs = set()
        for z in raw_wmap:
            for ap in z['aps']:
                macs.add(ap['MAC'])
        


    def localize(self, meas):
        """ 
        returns best guess of location and error
        meas should be list of dicts 
        """
        sims = [Similarity(loc, sim_cosine(meas, aps)) for loc, aps in self.wmap.items()]
        most_sim = heapq.nlargest(10, sims)

        for sim in most_sim:
            print(sim)

    


