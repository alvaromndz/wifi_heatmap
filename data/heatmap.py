import argparse
import json
import matplotlib.pyplot as plt
import numpy as np
import re
from scipy.interpolate import griddata

def read_pgm(filename, byteorder='>'):
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


def main(args):
    # settings
    fname = args.filename
    AP = args.AP
    contour_steps = 300

    lx = []
    ly = []
    llink = []
    with open(fname, "r") as read_file:
        data = json.load(read_file)
        for line in data:
            strens = [link['strength'] for link in line['strengths'] 
                            if link['SSID'] == AP]

            if len(strens) > 0:
                lx.append(line['x'])
                ly.append(line['y'])
                llink.append(max(strens))

        # for a square image both sizes must be equal
        minimum = min(lx) if min(lx) < min(ly) else min(ly)
        maximum = max(lx) if max(lx) > max(ly) else max(ly)
        if abs(minimum) > abs(maximum):
            maximum = -abs(minimum)
        else:
            minimum = -abs(maximum)

        lx = np.array(lx)
        ly = np.array(ly)
        llink = np.array(llink)


    #%%
    imgSize = [2048, 2048]
    imgResolution = 0.050000
    imgOrigin = [-39.400000, -30.250000, 0.000000]


    def meterToPixel(m):
        return m/imgResolution 

    def pixelToMeter(p):
        return p * imgResolution

    #  Map views always need a projection.  Here we just want to map image
    #  coordinates directly to map coordinates, so we create a projection that uses
    #  the image extent in pixels.
    origin = [imgOrigin[0], imgOrigin[1]]
    extent = [origin[0], pixelToMeter(imgSize[0]) + origin[0], origin[1], pixelToMeter(imgSize[1]) + origin[1]]
    fig = plt.figure(num=None, figsize=(20, 20))
    ax=fig.add_axes((0,0,1,1))

    plt.imshow(read_pgm('2.pgm'), extent=extent, cmap=plt.get_cmap('Blues_r'))

    xi = np.linspace(minimum, maximum, contour_steps)
    yi = np.linspace(minimum, maximum, contour_steps)
    zi = griddata((lx, ly), llink, (xi[None,:], yi[:,None]), method='linear')
    plt.contourf(xi, yi, zi, cmap="RdYlGn", vmin=np.nanmin(zi), vmax=np.nanmax(zi), alpha=.4)
    # plt.contourf(xi, yi, zi, cmap="RdYlGn", vmin=0., vmax=1., alpha=.4)
    plt.scatter(lx, ly, s=33, c='k', alpha=.7)
    ax.set_axis_off()
    plt.savefig('heatmap.png', transparent=True)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='make heatmap from json')
    parser.add_argument('filename', type=str, help='json file to use for heatmap')
    parser.add_argument('AP', type=str, help='AP to use for heatmap')
    args = parser.parse_args()
    main(args)
