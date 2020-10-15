import numpy as np
import configparser

config = configparser.ConfigParser()
config.read('config.ini')

WIN_WIDTH = int(config['window']['width'])
WIN_HEIGHT = int(config['window']['height'])

class polar_pos_t():
    '''
    Coordinates of window in polar form. Origin is the bottom center point
    of the window.
    '''
    def __init__(self, r = 0, theta = 0):
        self.r, self.theta = r, theta
    def to_cart(self):
        x = self.r * np.cos(self.theta)
        y = self.r * np.sin(self.theta)
        return cart_pos_t(x,y)
    def to_win(self):
        cart = self.to_cart()
        x, y = cart.x, cart.y
        y = WIN_HEIGHT - y
        x = WIN_WIDTH/2 + x
        return win_pos_t(x, y)

class cart_pos_t():
    '''
    Coordinates in cartesian form. Origin is the bottom center point of
    simulation window.
    '''
    def __init__(self, x = 0, y = 0):
        self.x, self.y = x, y
    def to_polar(self):
        r = np.sqrt(self.x*self.x + self.y*self.y)
        theta = np.arctan2(self.y, self.x)
        return polar_pos_t(r, theta)
    def to_win(self):
        x, y = self.x, self.y
        x += WIN_WIDTH/2
        y = -y + WIN_HEIGHT
        return win_pos_t(x, y)

class win_pos_t():
    '''
    Coordinates in matrix form. Origin is at top left of simulation window,
    x coord increases moving right, y increases moving down.
    '''
    def __init__(self, x = 0, y = 0):
        self.x, self.y = x, y
    def to_cart(self):
        x = self.x - WIN_WIDTH/2
        y = -(self.y - WIN_HEIGHT)
        return cart_pos_t(x, y)
    def to_polar(self):
        cart = self.to_cart()
        r = np.sqrt(cart.x*cart.x + cart.y*cart.y)
        theta = np.arctan2(cart.y, cart.x)
        return polar_pos_t(r, theta)

class vector_field_t():
    '''
    Defines the different types of vector fields available
    '''
    def __init__(self, center = cart_pos_t(0,0), desc = {}, kind = 0):
        self.center = center
        self.desc = desc
        self.kind = kind
        self.field_t = {
                'CIRCLE': 0,
                'CIRCLEBOUND': 1,
                'SPIRAL': 2,
                'SPIRALBOUND': 3
                }
        self.field_funcs = {
                self.field_t['CIRCLE']      : self._circle,
                self.field_t['CIRCLEBOUND'] : self._circle_bound,
                self.field_t['SPIRAL']      : self._spiral,
                self.field_t['SPIRALBOUND'] : self._spiral_bound,
                }
        self.selection = None

    def usr_input(self):
        '''
        Grab user input for vector field type
        '''
        print("Possible field types:")
        print("\t(0) Circle\n\t(1) Circle Bound\n\t(2) Spiral\n\t(3) Spiral Bound")
        try:
            selection = int(input("Selection: "))
            if selection > 3 or selection < 0:
                raise ValueError('Selection out of bounds')
            self.kind = selection
        except:
            raise

    def define(self, arm, selection = None):
        '''
        Change the vector field kind and set needed parameters
        '''
        if selection is not None:
            self.kind = selection
        self.desc = {}
        selection = self.kind
        if selection == self.field_t['CIRCLE']:
            self.center = cart_pos_t(arm.length1, arm.length0)
            self.desc["dtheta"] = np.pi / 2
        elif selection == self.field_t['CIRCLEBOUND']:
            self.center = cart_pos_t(arm.length1, arm.length0)
            self.desc["dr"] = np.pi/4
            self.desc["radius"] = arm.length1 / 2
            self.desc["buffer"] = arm.length1 / 20
        elif selection == self.field_t['SPIRAL']:
            self.center = cart_pos_t(arm.length1, arm.length0)
            self.desc["dtheta"] = np.pi / 2
            self.desc["dr"] = np.pi/4
        elif selection == self.field_t['SPIRALBOUND']:
            self.center = cart_pos_t(arm.length1, arm.length0)
            self.desc["dtheta"] = np.pi / 2
            self.desc["dr"] = arm.length1 / 8
            self.desc["radius"] = arm.length1 / 2
            self.desc["buffer"] = arm.length1 / 20
    def update_arm(self, arm0, arm1, elapsed_time):
        '''
        Calculate new arm positions based on vector field and redraw accordingly
        '''
        return self.field_funcs[self.kind](arm0, arm1, elapsed_time)
    def _circle(self, arm0, arm1, elapsed_time):
        cpos = fwd_kinematics(arm0, arm1)
        cpos = shift_origin(cpos, self.center)
        ppos = cpos.to_polar()
        dx, dy = polar_to_cart_d(ppos.r, ppos.theta, 0, self.desc["dtheta"])
        X = np.matrix([[dx],[dy]])
        jinv = inv_jacobian(arm0, arm1)
        result = np.matmul(jinv, X)
        dtheta0 = result[0,0]
        dtheta1 = result[1,0]
        arm0.rot += rad_to_deg(dtheta0) * elapsed_time
        arm1.ppos = arm0.get_end()
        arm1.rot += rad_to_deg(dtheta1) * elapsed_time
        self.dx, self.dy = dx, dy
    def _circle_bound(self, arm0, arm1, elapsed_time):
        cpos = fwd_kinematics(arm0, arm1)
        cpos = shift_origin(cpos, self.center)
        ppos = cpos.to_polar()
        dr = self.desc["dr"]
        if ppos.r > self.desc["radius"]+self.desc["buffer"]:
            dr *= 1
        elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
            dr *= -1
        dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, 0)
        X = np.matrix([[dx],[dy]])
        jinv = inv_jacobian(arm0, arm1)
        result = np.matmul(jinv, X)
        dtheta0 = result[0,0]
        dtheta1 = result[1,0]
        arm0.rot += rad_to_deg(dtheta0) * elapsed_time
        arm1.ppos = arm0.get_end()
        arm1.rot += rad_to_deg(dtheta1) * elapsed_time
        self.dx, self.dy = dx, dy
    def _spiral(self, arm0, arm1, elapsed_time):
        cpos = fwd_kinematics(arm0, arm1)
        cpos = shift_origin(cpos, self.center)
        ppos = cpos.to_polar()
        dx, dy = polar_to_cart_d(ppos.r, ppos.theta, self.desc["dr"], self.desc["dtheta"])
        X = np.matrix([[dx],[dy]])
        jinv = inv_jacobian(arm0, arm1)
        result = np.matmul(jinv, X)
        dtheta0 = result[0,0]
        dtheta1 = result[1,0]
        arm0.rot += rad_to_deg(dtheta0) * elapsed_time
        arm1.ppos = arm0.get_end()
        arm1.rot += rad_to_deg(dtheta1) * elapsed_time
        self.dx, self.dy = dx, dy
    def _spiral_bound(self, arm0, arm1, elapsed_time):
        cpos = fwd_kinematics(arm0, arm1)
        cpos = shift_origin(cpos, self.center)
        ppos = cpos.to_polar()
        dr = self.desc["dr"]
        if ppos.r > self.desc["radius"]+self.desc["buffer"]:
            dr *= 1
        elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
            dr *= -1
        dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, self.desc["dtheta"])
        X = np.matrix([[dx],[dy]])
        jinv = inv_jacobian(arm0, arm1)
        result = np.matmul(jinv, X)
        dtheta0 = result[0,0]
        dtheta1 = result[1,0]
        arm0.rot += rad_to_deg(dtheta0) * elapsed_time
        arm1.ppos = arm0.get_end()
        arm1.rot += rad_to_deg(dtheta1) * elapsed_time
        self.dx, self.dy = dx, dy
    def get_arrow_rot(self, cpos):
        '''
        Calculates rotation of arrows for showing vector field
        '''
        selection = self.kind
        if selection == self.field_t['CIRCLE']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, 0, self.desc["dtheta"])
            return dx, dy
        elif selection == self.field_t['CIRCLEBOUND']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dr = self.desc["dr"]
            if ppos.r > self.desc["radius"]+self.desc["buffer"]:
                dr *= 1
            elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
                dr *= -1
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, 0)
            return dx, dy
        elif selection == self.field_t['SPIRAL']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, self.desc["dr"], self.desc["dtheta"])
            return dx, dy
        elif selection == self.field_t['SPIRALBOUND']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dr = self.desc["dr"]
            if ppos.r > self.desc["radius"]+self.desc["buffer"]:
                dr *= 1
            elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
                dr *= -1
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, self.desc["dtheta"])
            return dx, dy
        return np.finfo(float).max, np.finfo(float).max

def shift_origin(cpos, new_origin):
    '''
    Translate point based on new origin
    '''
    return cart_pos_t(cpos.x - new_origin.x, cpos.y - new_origin.y)

def deg_to_rad(deg):
    '''
    Convert degrees to radians
    '''
    return deg * np.pi / 180

def rad_to_deg(rad):
    '''
    Convert radians to degrees
    '''
    return -(rad * 180 / np.pi)

def fwd_kinematics(arm0, arm1):
    '''
    Calculate end effector position using forward kinematics
    '''
    x = arm0.length*np.cos(deg_to_rad(arm0.rot)) + arm1.length*np.cos(deg_to_rad(arm1.rot))
    y = arm0.length*np.sin(deg_to_rad(arm0.rot)) + arm1.length*np.sin(deg_to_rad(arm1.rot))
    return cart_pos_t(x, y)

def jacobian(arm0, arm1):
    '''
    Return jacobian matrix of arm
    '''
    j = np.matrix(
            [[-arm0.length*np.sin(deg_to_rad(arm0.rot)), -arm1.length*np.sin(deg_to_rad(arm1.rot))],
            [arm0.length*np.cos(deg_to_rad(arm0.rot)), arm1.length*np.cos(deg_to_rad(arm1.rot))]]
            )
    return j

def inv_jacobian(arm0, arm1):
    '''
    Invert jacobian
    '''
    j = jacobian(arm0, arm1)
    jinv = np.linalg.pinv(j)
    return jinv

def polar_to_cart_d(r, theta, dr, dtheta):
    '''
    Convert polar velocities to cartesian velocities
    '''
    dx = dr*np.cos(theta) - r*np.sin(theta)*dtheta
    dy = dr*np.sin(theta) + r*np.cos(theta)*dtheta
    return dx, dy

def count_to_theta(arm, count0, count1):
    """
    Convert encoder counts to theta, must be passed current arm class
    """
    if arm.calibrated:
        theta0 = (count0 - arm.count0_zero) / arm.cnt_per_rad0
        theta1 = (count1 - arm.count1_zero) / arm.cnt_per_rad1

        return theta0, theta1
    else:
        raise Exception("Calibration routine for arm must be run first")

def theta_to_count(arm, theta0, theta1):
    """
    Convert radians to encoder counts, must be passed arm class
    """
    if arm.calibrated:
        count0 = theta0 * arm.cnt_per_rad0 + arm.count0_zero
        count1 = theta1 * arm.cnt_per_rad1 + arm.count1_zero

        return count0, count1
    else:
        raise Exception("Calibration routine for arm must be run first")

