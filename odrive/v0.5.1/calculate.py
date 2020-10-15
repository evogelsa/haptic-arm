import numpy as np

def cart2polar(x, y):
    r = ((x**2) + (y**2))**.5
    theta = np.arctan(y/x)

    return (r, theta)

class VectorField():
    def __init__(self, field='spiralbound', **kwargs):
        self.field = field
        self.args = kwargs
    def circlebound(self):
        try:
            radius = self.args['radius']
            buffer = self.args['buffer']
            x_center = self.args['x_center']
            y_center = self.args['y_center']
        except KeyError:
            print('Missing required arguments for \'circlebound\'')
            raise

            outer = radius + buffer/2
            inner = radius - buffer/2

            x -= x_center
            y -= y_center

            r, theta = cart2polar(x, y)
