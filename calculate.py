import warnings
import numpy as np
from collections import namedtuple

CartesianCoordinates = namedtuple('CartesianCoordinates','x y')
WindowCoordinates = namedtuple('WindowCoordinates','i j')
PolarCoordinates = namedtuple('PolarCoordinates', 'r theta')

PIXELS_PER_METER = 1000


def cart2polar(x, y):
    '''
    Convert cartesian coordinates to polar
    '''
    r = ((x**2) + (y**2))**.5
    theta = np.arctan2(y, x)
    return (r, theta)

def dpolar2cart(r, theta, dr, dtheta):
    '''
    Convert polar velocities to cartesian
    '''
    dx = dr * np.cos(theta) - r * np.sin(theta) * dtheta
    dy = dr * np.sin(theta) + r * np.cos(theta) * dtheta
    return dx, dy

def map(v, vmin, vmax, tmin, tmax):
    '''
    Map a given value in a range [vmin,vmax] to a target range [tmin,tmax]
    '''
    v = min(v, vmax)
    v = max(v, vmin)
    vrange = vmax - vmin
    vscale = (v - vmin) / vrange
    trange = tmax - tmin
    tv = vscale * trange + tmin
    return tv

def count2rad(arm, count, axis):
    '''
    Convert encoder counts to radians
    '''
    warnings.warn(
            'calculate.count2rad deprecated: use device method instead',
            DeprecationWarning
    )
    try:
        if axis == 0:
            return (count - arm.arm0.zero) / arm.arm0.cnt_per_rad
        elif axis == 1:
            return (count - arm.arm1.zero) / arm.arm1.cnt_per_rad
    except TypeError:
        print('Unable to convert counts to radians; arm may not be calibrated')
        raise

def rad2count(arm, theta, axis):
    '''
    Convert radians to encoder counts
    '''
    warnings.warn(
            'calculate.rad2count depcrated: use device method instead',
            DeprecationWarning
    )
    try:
        if axis == 0:
            return theta * arm.arm0.cnt_per_rad + arm.arm0.zero
        elif axis == 1:
            return theta * arm.arm1.cnt_per_rad + arm.arm1.zero
    except TypeError:
        print('Unable to convert radians to counts; arm may not be calibrated')
        raise

def fwd_kinematics(arm, theta0, theta1):
    '''
    Calculate the end effector position in cartesian coordinates from the
    given arm configuration
    '''
    warnings.warn(
            'calculate.fwd_kinematics deprecated: use device method',
            DeprecationWarning
    )
    x = arm.arm0.length * np.cos(theta0) + arm.arm1.length * np.cos(theta1)
    y = arm.arm0.length * np.sin(theta0) + arm.arm1.length * np.sin(theta1)
    return x, y

def jacobian(arm, theta0, theta1):
    '''
    Return the jacobian matrix from the given arm angles
    '''
    warnings.warn(
            'calculate.jacobain deprecated: use device method',
            DeprecationWarning
    )
    return np.array([[-arm.arm0.length * np.sin(theta0),
                      -arm.arm1.length * np.sin(theta1)],
                     [arm.arm0.length * np.cos(theta0),
                      arm.arm1.length * np.cos(theta1)]])

def inv_jacobian(arm, theta0, theta1):
    '''
    Return the inverted jacobian matrix
    '''
    warnings.warn(
            'calculate.inv_jacobian deprecated: use device method',
            DeprecationWarning
    )
    return np.linalg.pinv(jacobian(arm, theta0, theta1))

class Coord():
    '''
    Coord is a class to store position data on the visualization in different
    forms. Stores cartesian, polar, and window (graphics) coordinates and auto
    updates each form when one is set
    '''
    def __init__(self, cpos=None, wpos=None, ppos=None,
                 win_width=800, win_height=600):
        '''
        When a coord is initialized, if multiple kinds of coordinates are
        passed, cartesian coordinates are given priority.
        '''
        self._cpos = None
        self._wpos = None
        self._ppos = None
        self.win_width = win_width
        self.win_height = win_height
        if cpos is not None:
            try:
                self._cpos = CartesianCoordinates(cpos[0], cpos[1])
            except:
                print('Error setting initial value for cartesian coordinates.')
                raise
        elif wpos is not None:
            try:
                self._wpos = WindowCoordinates(wpos[0], wpos[1])
            except:
                print('Error setting initial value for window coordinates.')
                raise
        elif ppos is not None:
            try:
                self._ppos = PolarCoordinates(ppos[0], ppos[1])
            except:
                print('Error setting initial value for polar coordinates')
                raise

        if self._cpos is None and self._wpos is None and self._ppos is None:
            raise UserWarning('No initial values supplied for position')

        if self._cpos is not None:
            self._set_polar_from_cart()
            self._set_window_from_cart()
        elif self._wpos is not None:
            self._set_cart_from_window()
            self._set_polar_from_window()
        elif self._ppos is not None:
            self._set_cart_from_polar()
            self._set_window_from_polar()

    def __repr__(self):
        cpos = f'Cartesian(x={self.cartesian.x}, y={self.cartesian.y})'
        wpos = f'Window(i={self.window.i}, j={self.window.j})'
        ppos = f'Polar(r={self.polar.r}, theta={self.polar.theta})'
        coord = f'Coordinate({cpos}, {wpos}, {ppos})'
        return coord

    def __eq__(self, other):
        x = (self.cartesian.x == other.cartesian.x)
        y = (self.cartesian.y == other.cartesian.y)
        return (x and y)

    def _set_polar_from_cart(self):
        #  a = self._cpos.x + 1j * self._cpos.y
        #  r = np.abs(a)
        #  theta = np.angle(a)
        r = ((self._cpos.x**2) + (self._cpos.y**2))**0.5
        theta = np.arctan2(self._cpos.y, self._cpos.x)
        self._ppos = PolarCoordinates(r, theta)

    def _set_window_from_cart(self):
        i = self._cpos.x*PIXELS_PER_METER
        j = (self._cpos.y*PIXELS_PER_METER) + self.win_width/2
        self._wpos = WindowCoordinates(i, j)

    def _set_cart_from_window(self):
        x = (self._wpos.i)/PIXELS_PER_METER
        y = (self._wpos.j - self.win_width/2)/PIXELS_PER_METER
        self._cpos = CartesianCoordinates(x, y)

    def _set_polar_from_window(self):
        self._set_cart_from_window()
        self._set_polar_from_cart()

    def _set_cart_from_polar(self):
        x = self._ppos.r * np.cos(self._ppos.theta)
        y = self._ppos.r * np.sin(self._ppos.theta)
        self._cpos = CartesianCoordinates(x, y)

    def _set_window_from_polar(self):
        self._set_cart_from_polar()
        self._set_window_from_cart()

    @property
    def cartesian(self):
        return self._cpos

    @cartesian.setter
    def cartesian(self, pointpair):
        self._cpos = CartesianCoordinates(*pointpair)
        self._set_window_from_cart()
        self._set_polar_from_cart()

    @property
    def window(self):
        if type(self._wpos.i) == np.ndarray and type(self._wpos.j) == np.ndarray:
            vectorint = np.vectorize(np.int)
            return WindowCoordinates(vectorint(self._wpos.i),
                                     vectorint(self._wpos.j))
        else:
            return WindowCoordinates(int(self._wpos.i+0.5),
                                     int(self._wpos.j+0.5))

    @window.setter
    def window(self, pointpair):
        self._wpos = WindowCoordinates(*pointpair)
        self._set_cart_from_window()
        self._set_polar_from_window()

    @property
    def polar(self):
        return self._ppos

    @polar.setter
    def polar(self, pointpair):
        self._ppos = PolarCoordinates(*pointpair)
        self._set_cart_from_polar()
        self._set_window_from_polar()

    @property
    def x(self):
        return self._cpos.x

    @x.setter
    def x(self, cartx):
        self._cpos = CartesianCoordinates(cartx, self._cpos.y)
        self._set_window_from_cart()
        self._set_polar_from_cart()

    @property
    def y(self):
        return self._cpos.y

    @y.setter
    def y(self, carty):
        self._cpos = CartesianCoordinates(self._cpos.x, carty)
        self._set_window_from_cart()
        self._set_polar_from_cart()

    @property
    def i(self):
        return self._wpos.i

    @i.setter
    def i(self, windowi):
        self._wpos = WindowCoordinates(windowi, self._wpos.j)
        self._set_cart_from_window()
        self._set_polar_from_window()

    @property
    def j(self):
        return self._wpos.j

    @j.setter
    def j(self, windowj):
        self._wpos = WindowCoordinates(self._wpos.i, windowj)
        self._set_cart_from_window()
        self._set_polar_from_window()

    @property
    def r(self):
        return self._ppos.r

    @r.setter
    def r(self, polarr):
        self._ppos = PolarCoordinates(polarr, self._ppos.theta)
        self._set_cart_from_polar()
        self._set_window_from_polar()

    @property
    def theta(self):
        return self._ppos.theta

    @theta.setter
    def theta(self, polartheta):
        self._ppos = PolarCoordinates(self._ppos.r, polartheta)
        self._set_cart_from_polar()
        self._set_window_from_polar()


class VectorField():
    def __init__(self, arm, field='spiralbound', args=None):
        self.field = field
        self.args = args
        self.arm = arm
        self._fields = {
                'circle'      : self.circle,
                'circlebound' : self.circlebound,
                'spiralbound' : self.spiralbound,
                'spring' : self.spring,
                }

    def return_vectors(self, x, y):
        '''Return vectors appropriate to field type'''
        if self.field is None or self.args is None:
            raise ValueError('Field and arguments must be defined before'
                             + ' vectors can be returned')
        return self._fields[self.field](x, y)

    def spring(self, x, y):
        try:
            xcenter = self.args['xcenter']
            ycenter = self.args['ycenter']
            drmax   = self.args['drmax']
        except KeyError:
            print('Missing required arguments for \'circle\'')
            raise
        x -= xcenter
        y -= ycenter

        r, theta = cart2polar(x, y)
        dr = map(r, 0, self.arm.arm0.length, 0, -drmax)

        dx, dy = dpolar2cart(r, theta, dr, 0)

        return dx, dy

    def circle(self, x, y):
        try:
            xcenter = self.args['xcenter']
            ycenter = self.args['ycenter']
            dtheta  = self.args['dtheta']
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
            radius  = self.args['radius']
            buffer  = self.args['buffer']
            xcenter = self.args['xcenter']
            ycenter = self.args['ycenter']
            drmax   = self.args['drmax']
        except KeyError:
            print('Missing required arguments for \'circlebound\'')
            raise

        outer = radius + buffer/2
        inner = radius - buffer/2

        x -= xcenter
        y -= ycenter

        r, theta = cart2polar(x, y)

        dr = 0
        if r > outer:
            vel_eq = (r - outer)
            # vel_eq is linear based on distance from outer circle boundary
            # so map this to a reasonable range for the velocity
            dr = map(vel_eq, 0, self.arm.arm0.length +
                     self.arm.arm1.length, 0, -drmax)
        elif r < inner:
            vel_eq = inner - r
            dr = map(vel_eq, 0, self.arm.arm0.length +
                     self.arm.arm1.length, 0, drmax)

        dx, dy = dpolar2cart(r, theta, dr, 0)
        return dx, dy

    def spiralbound(self, x, y):
        dx0, dy0 = self.circlebound(x, y)
        dx1, dy1 = self.circle(x, y)
        dx = dx0 + dx1
        dy = dy0 + dy1
        return dx, dy
