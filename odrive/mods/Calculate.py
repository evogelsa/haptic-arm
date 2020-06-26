import numpy as np


def torque(odrv0, axis):
    """
    torque measurements give best fit estimate for peak torque when motor cool:
    .417(amps) - 4.2*10^-3
    Estimate only, probably accurate within about 10% error
    """
    if axis == 0:
        return (.417 * odrv0.axis0.motor.current_control.Iq_measured
                - 4.2*10**-3)
    elif axis == 1:
        return (.417 * odrv0.axis1.motor.current_control.Iq_measured
                - 4.2*10**-3)
    else:
        raise Exception("Trying to calculate torque of nonexistant axis %d"
                        %axis)


def position(odrv0, axis):
    """
    Return position of axis using built in odrive shadow count
    """
    if axis == 0:
        return odrv0.axis0.encoder.shadow_count
    elif axis == 1:
        return odrv0.axis1.encoder.shadow_count
    else:
        raise Exception("Trying to calculate position of nonexistant axis %d"
                        %axis)


def velocity(odrv0, axis):
    """
    Use built in odrive velocity estimator to return velocity in radians
    """
    if axis == 0:
        velcps = odrv0.axis0.encoder.vel_estimate
        velrad = velcps * 2 * np.pi / 8192
        return velrad
    elif axis == 1:
        velcps = odrv0.axis1.encoder.vel_estimate
        velrad = velcps * 2 * np.pi / 8192
        return velrad
    else:
        raise Exception("Trying to calculate velocity of nonexistant axis %d"
                        %axis)


def count2theta(count0, count1, arm):
    """
    Convert encoder counts to theta, must be passed current arm class
    """
    if arm.calibrated:
        theta0 = (count0 - arm.count0_zero) / arm.cnt_per_rad0
        theta1 = (count1 - arm.count1_zero) / arm.cnt_per_rad1

        return theta0, theta1
    else:
        raise Exception("Calibration routine for arm must be run first")


def theta2count(theta0, theta1, arm):
    """
    Convert radians to encoder counts, must be passed arm class
    """
    if arm.calibrated:
        count0 = theta0 * arm.cnt_per_rad0 + arm.count0_zero
        count1 = theta1 * arm.cnt_per_rad1 + arm.count1_zero

        return count0, count1
    else:
        raise Exception("Calibration routine for arm must be run first")


def fwd_kinematics(theta0, theta1, arm):
    """
    Use forward kinematics to calculate end effector cartesian coordinate
    """
    x = arm.l1*np.cos(theta0) + arm.l2*np.cos(theta1)
    y = arm.l1*np.sin(theta0) + arm.l2*np.sin(theta1)

    return x, y


def jacobian(theta0, theta1, arm):
    """
    Return array of fwd kinematic eqns
    """
    return np.array([[-arm.l1 * np.sin(theta0), -arm.l2 * np.sin(theta1)],
                  [arm.l1 * np.cos(theta0), arm.l2 * np.cos(theta1)]])


def inv_jacobian(theta0, theta1, arm):
    """
    Returns inverse of jacobian
    """
    return np.linalg.inv(jacobian(theta0, theta1, arm))


def cart2polar(x, y):
    r = (x**2 + y**2)**.5
    theta = np.arctan(y / x)

    return r, theta


def polar2cart(r, theta):
    x = r*np.cos(theta)
    y = r*np.sin(theta)

    return x, y


def d_cart2polar(x, y, x_dot, y_dot):
    pass


def d_polar2cart(r, theta, r_dot, theta_dot):
    x_dot = r_dot * np.cos(theta) - r * np.sin(theta) * theta_dot
    y_dot = r_dot * np.sin(theta) + r * np.cos(theta) * theta_dot

    return x_dot, y_dot


class vector_fields():
    """
    Container for various vector fields
    """
    def __init__(self):
        self.fields = {
            'restrict_line': self._restrict_line,
            'restrict_circle': self._restrict_circle,
            'make_circle': self._make_circle,
            'return_to_circle': self._return_to_circle
        }
        self.defs = {
            'restrict_line': 'Vector field pushes back to line parallel to' \
                             + ' specified axis',
            'restrict_circle': 'Restrict arm movement to circle',
            'make_circle': 'Arm makes circles at constant theta velocity',
            'return_to_circle': 'Vector field spirals arm back to circle'
        }
        self.field = None
        self.args = None


    def return_field(self, x, y):
        """
        Returns desired vectors from field specified by selection
        """
        if self.field is None or self.args is None:
            raise ValueError('Must select field and arguments before a field'
                             + ' can be returned')
        return self.fields[self.field](x, y, **self.args)


    def select_field_by_args(self, field, **kwargs):
        """
        Select field through functions arguments
        """
        self.field = field
        self.args = kwargs

        return field, kwargs


    def select_field_by_user(self):
        """
        Get user to select field and func specific arguments
        """
        print("Select vector field: ")
        for key in self.fields.keys():
            print('\t' + str(key) + ': ' + self.defs[key])
        valid = False
        while True:
            field = str(input("Selection: "))
            if field in self.fields.keys():
                break
            else:
                print("Invalid key")

        # define func specific arguments
        args = {}
        if field == 'restrict_line':
            args['axis'] = None
            while args['axis'] is None:
                axis = str(input('Axis (x/y): ')).lower()
                if axis == 'x':
                    args['axis'] = 'x'
                elif axis == 'y':
                    args['axis'] = 'y'
                elif axis == '':
                    args['axis'] = 'x'
                    print('Defaulting to x')
            args['offset'] = None
            while args['offset'] is None:
                try:
                    inp = input('Axis offset (meters): ')
                    if inp == '':
                        args['offset'] = 0
                        print('Defaulting to 0')
                    else:
                        args['offset'] = float(inp)
                except:
                    pass
        elif field == 'restrict_circle':
            args['radius'] = None
            while args['radius'] is None:
                try:
                    inp = input('Radius (meters): ')
                    if inp == '':
                        args['radius'] = .05
                        print('Defaulting to .05')
                    else:
                        args['radius'] = float(inp)
                except:
                    pass
            args['buffer'] = None
            while args['buffer'] is None:
                try:
                    inp = input('Buffer (meters): ')
                    if inp == '':
                        args['buffer'] = .005
                        print('Defaulting to .005')
                    else:
                        args['buffer'] = float(inp)
                except:
                    pass
            args['center_x'] = None
            while args['center_x'] is None:
                try:
                    inp = input('Center x coord (meters): ')
                    if inp == '':
                        args['center_x'] = .1225
                        print('Defaulting to .1225')
                    else:
                        args['center_x'] = float(inp)
                except:
                    pass
            args['center_y'] = None
            while args['center_y'] is None:
                try:
                    inp = input('Center y coord (meters): ')
                    if inp == '':
                        args['center_y'] = .1225
                        print('Defaulting to .1225')
                    else:
                        args['center_y'] = float(inp)
                except:
                    pass
        elif field == 'make_circle':
            args['theta_dot'] = None
            while args['theta_dot'] is None:
                try:
                    inp = input('Theta vel (rad/s): ')
                    if inp == '':
                        args['theta_dot'] = np.pi/4
                        print('Defaulting to pi/4')
                    else:
                        args['theta_dot'] = float(inp)
                except:
                    pass
        elif field == 'return_to_circle':
            args['radius'] = None
            while args['radius'] is None:
                try:
                    inp = input('Radius (meters): ')
                    if inp == '':
                        args['radius'] = .05
                        print('Defaulting to .05')
                    else:
                        args['radius'] = float(inp)
                except:
                    pass
            args['buffer'] = None
            while args['buffer'] is None:
                try:
                    inp = input('Buffer (meters): ')
                    if inp == '':
                        args['buffer'] = .005
                        print('Defaulting to .005')
                    else:
                        args['buffer'] = float(inp)
                except:
                    pass
            args['center_x'] = None
            while args['center_x'] is None:
                try:
                    inp = input('Center x coord (meters): ')
                    if inp == '':
                        args['center_x'] = .1225
                        print('Defaulting to .1225')
                    else:
                        args['center_x'] = float(inp)
                except:
                    pass
            args['center_y'] = None
            while args['center_y'] is None:
                try:
                    inp = input('Center y coord (meters): ')
                    if inp == '':
                        args['center_y'] = .1225
                        print('Defaulting to .1225')
                    else:
                        args['center_y'] = float(inp)
                except:
                    pass
            args['theta_dot'] = None
            while args['theta_dot'] is None:
                try:
                    inp = input('Theta vel (rad/s): ')
                    if inp == '':
                        args['theta_dot'] = np.pi/4
                        print('Defaulting to pi/4')
                    else:
                        args['theta_dot'] = float(inp)
                except:
                    pass

        self.field = field
        self.args = args

        return field, args


    def _scale_velocity(self, v, v_min, v_max, map_min, map_max):
        """
        Helper function to map given velocity in given to range to new range
        """
        v_range = v_max - v_min
        v_scale = (v - v_min) / v_range
        map_range = map_max - map_min
        v = (v_scale * map_range) + map_min

        return v


    def _restrict_line(self, x, y, **kwargs):
        """
        Describes a vector field which directs arm to stay on line parallel to
        given axis, offset by given offset
        """
        axis = kwargs['axis']
        offset = kwargs['offset']

        if axis == 'x':
            x_dot = 0
            # function of y maps y displacement to y_dot range [0 A to 1 A]
            y_dot = _scale_velocity(y - offset, 0, l1*2, 0, .03)
        elif axis == 'y':
            # function of x maps x displacement to x_dot range [0 A to 1 A]
            x_dot = _scale_velocity(x - offset, 0, l1*2, 0, .03)
            y_dot = 0

        return x_dot, y_dot


    def _restrict_square(self, x, y, **kwargs):
        """
        Describe vector field that restricts arm movements to square
        """
        #TODO
        center_x = kwargs['center_x']
        center_y = kwargs['center_y']
        length = kwargs['length']

        return None

    def _restrict_circle(self, x, y, **kwargs):
        """
        Describes vector field which returns arm back to a circle defined by
        given parameters.
        """
        radius = kwargs['radius']
        buffer = kwargs['buffer']
        center_x = kwargs['center_x']
        center_y = kwargs['center_y']

        outter = radius + buffer
        inner = radius - buffer

        x -= center_x
        y -= center_y

        r, theta = cart2polar(x, y)

        if r > outter:
            vel_eq = -(r - outter)
            r_dot = _scale_velocity(vel_eq, 0, l1*2, 0, .03)
        elif r < inner:
            vel_eq = inner - r
            r_dot = _scale_velocity(vel_eq, 0, l1*2, 0, .03)

        theta_dot = 0

        x_dot, y_dot = d_polar2cart(r, theta, r_dot, theta_dot)

        return x_dot, y_dot


    def _make_circle(self, x, y, **kwargs):
        # TODO - this needs to take center into account
        r, theta = cart2polar(x, y)
        r_dot = 0
        # better way to set theta_dot? unsure what good velocity might be
        theta_dot = np.pi/4
        x_dot, y_dot = d_polar2cart(r, theta, r_dot, theta_dot)

        return x_dot, y_dot


    def _return_to_circle(self, x, y, **kwargs):
        x_dot0, y_dot0 = _restrict_circle(x, y, **kwargs)
        x_dot1, y_dot1 = _make_circle(x, y, **kwargs)

        x_dot = x_dot0 + x_dot1
        y_dot = y_dot0 + y_dot1

        return x_dot, y_dot
