import json
from collections import defaultdict
import numpy as np
import pandas as pd
from sklearn.metrics.pairwise import cosine_similarity

class WifiMap(object):
    def __init__(self, wmap_file):
        # load json
        with open(wmap_file, 'r') as f:
            wmap_json = json.load(f)

        # build mapping of location -> list of measurements at location
        wmap_dict = defaultdict(list)
        for m in wmap_json:
            aps = pd.Series({ap['MAC'] : ap['strength'] for ap in z['aps']})
            if len(aps) > 1: # if only one probably bad scan
                wmap_dict[(m['x'], m['y'], m['z'], m['w'])].append(aps)

        self.wmap_dict = {loc : pd.DataFrame(aps).mean() 
                             for loc, aps in wmap_dict.items()}
        
        self.wmap = pd.DataFrame(self.wmap_dict)
        self.wmap.fillna(0., inplace=True)
        self.locs = np.array([list(loc) for loc in self.wmap.columns])
    
    def probable_location(self, meas, n=5):
        if n <= 0 or n > self.wmap.columns.size:
            n = self.wmap.columns.size

        ssim = self.p_loc_meas(meas)
        most_likely = np.argpartition(-ssim, n-1)[:n]

        weights = ssim[most_likely]        
        norm = np.linalg.norm(weights, ord=1)
        plocs = weights / norm if norm > 0 else weights

        loc = (self.locs[most_likely, :] * plocs[:, None]).sum(axis=0)
        return loc
    
    def p_loc_meas(self, meas):
        X, Y = self.wmap.align(meas, fill_value=0, axis=0)
        return cosine_similarity(X.T, Y[None, :]).flatten()
