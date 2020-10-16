import numpy as np

def cart2polar(x, y):
    r = ((x**2) + (y**2))**.5
    theta = np.arctan(y/x)

    return (r, theta)

def dpolar2cart(r, theta, dr, dtheta):
    dx = dr * np.cos(theta) - r * np.sin(theta) * dtheta
    dy = dr * np.sin(theta) + r * np.cos(theta) * dtheta
    return dx, dy

def map(v, vmin, vmax, tmin, tmax):
    vrange = vmax - vmin
    vscale = (v - vmin) / vrange
    trange = tmax - tmin
    v = vscale * trange + tmin
    return v

def count2rad(arm, count, axis):
    try:
        if axis == 0:
            return (count - arm.arm0.zero) / arm.arm0.cnt_per_rad
        elif axis == 1:
            return (count - arm.arm1.zero) / arm.arm1.cnt_per_rad
    except TypeError:
        print('Unable to convert counts to radians; arm may not be calibrated')
        raise

def rad2count(arm, theta, axis):
    try:
        if axis == 0:
            return theta * arm.arm0.cnt_per_rad + arm.arm0.zero
        elif axis == 1:
            return theta * arm.arm1.cnt_per_rad + arm.arm1.zero
    except TypeError:
        print('Unable to convert radians to counts; arm may not be calibrated')
        raise

def fwd_kinematics(arm, theta0, theta1):
    x = arm.arm0.length * np.cos(theta0) + arm.arm1.length * np.cos(theta1)
    y = arm.arm1.length * np.sin(theta1) + arm.arm1.length * np.sin(theta1)
    return x, y

def jacobian(arm, theta0, theta1):
    return np.array([[-arm.arm0.length * np.sin(theta0),
                      -arm.arm0.length * np.sin(theta1)],
                    [arm.arm0.length * np.cos(theta0),
                     arm.arm1.length * np.cos(theta1)]])

def inv_jacobian(arm, theta0, theta1):
    return np.linalg.pinv(jacobian(arm, theta0, theta1))

class VectorField():
    def __init__(self, arm, field='spiralbound', args=None):
        self.field = field
        self.args = args
        self.arm = arm
        self._fields = {
                'circle'      : self.circle,
                'circlebound' : self.circlebound,
                'spiralbound' : self.spiralbound,
                }
    def return_vectors(self, x, y):
        if self.field is None or self.args is None:
            raise ValueError('Field and arguments must be defined before'
                             + ' vectors can be returned')
        return self._fields[self.field](x, y)
    def circle(self, x, y):
        try:
            xcenter = self.args['xcenter']
            ycenter = self.args['ycenter']
            dtheta = self.args['dtheta']
        except KeyError:
            print('Missing required arguments for \'circle\'')
            raise

        x -= xcenter
        y -= ycenter

        r, theta = cart2polar(x, y)
        dx, dy = dpolar2cart(r, theta, 0, dtheta)

        return dx, dy
    def circlebound(self, x, y):
        try:
            radius = self.args['radius']
            buffer = self.args['buffer']
            xcenter = self.args['xcenter']
            ycenter = self.args['ycenter']
        except KeyError:
            print('Missing required arguments for \'circlebound\'')
            raise

        outer = radius + buffer/2
        inner = radius - buffer/2

        x -= xcenter
        y -= ycenter

        r, theta = cart2polar(x, y)

        if r > outer:
            vel_eq = -(r - outer)
            # vel_eq is linear based on distance from outer circle boundary
            # so map this to a reasonable range for the velocity
            dr = map(vel_eq, 0, self.arm.arm0.length +
                     self.arm.arm1.length, 0, 0.03)
        elif r < inner:
            vel_eq = inner - r
            dr = map(vel_eq, 0, self.arm.arm0.length +
                     self.arm.arm1.length, 0, 0.03)

        dx, dy = dpolar2cart(r, theta, dr, 0)
        return dx, dy
    def spiralbound(self, x, y):
        dx0, dy0 = self.circlebound(x, y)
        dx1, dy1 = self.circle(x, y)
        dx = dx0 + dx1
        dy = dy0 + dy1
        return dx, dy
