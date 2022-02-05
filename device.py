import numpy as np
from time import sleep
import odrive
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_TORQUE_CONTROL,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
)


class HapticDevice:
    """
    Wrapper for the SCARA arm and its associated utility functions. Stores
    objects for each arm segment and the odrive controller
    """

    # TODO
    # [ ] create @property and @setter methods for arm angles
    # [ ] " " for x, y

    def __init__(self, init_with_device=True, arm_length=0.2):
        self.odrive = odrive.find_any() if init_with_device else None
        self.arm0 = ArmSegment(arm_length)
        self.arm1 = ArmSegment(arm_length)

    def calibrate(self, axes=[0, 1]):
        """
        Run the odrive axis/encoder calibration routinen on the specified axes
        """
        if 0 in axes:
            print('Calibrating axis 0')
            self.odrive.axis0.requested_state = (
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            )
            while self.odrive.axis0.current_state != AXIS_STATE_IDLE:
                sleep(0.1)
        if 1 in axes:
            print('Calibrating axis 1')
            self.odrive.axis1.requested_state = (
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            )
            while self.odrive.axis1.current_state != AXIS_STATE_IDLE:
                sleep(0.1)

    def home(self):
        """
        Run the homing routine to calculate zero position and equivalent encoder
        counts to radians/degrees. In the future can be automated with endstops
        """
        # move to zero position and record
        input("Move arm to first position and press enter to continue")
        self.arm0.zero = self.odrive.axis0.encoder.shadow_count
        self.arm1.zero = self.odrive.axis1.encoder.shadow_count
        print(self.arm0.zero, self.arm1.zero)

        # move to defined location and record
        input("Move arm 90 deg counterclockwise and press enter to continue")
        cal0 = self.odrive.axis0.encoder.shadow_count
        cal1 = self.odrive.axis1.encoder.shadow_count
        print(cal0, cal1)

        # calculate the difference in encoder counts and define conversion ratio
        # to radians
        theta_dif = np.radians(90)
        cnt0_dif = cal0 - self.arm0.zero
        cnt1_dif = cal1 - self.arm1.zero
        print(cnt0_dif, cnt1_dif)

        self.arm0.cnt_per_rad = cnt0_dif / theta_dif
        self.arm1.cnt_per_rad = cnt1_dif / theta_dif
        print(self.arm0.cnt_per_rad, self.arm1.cnt_per_rad)

    def set_axis_state_closed_loop(self, axes=[0, 1]):
        """
        Set the specified axes to closed loop control mode, requires
        calibration. Automatically called when setting control modes
        """
        if 0 in axes:
            if (
                self.odrive.axis0.motor.is_calibrated
                and self.odrive.axis0.encoder.is_ready
            ):
                self.odrive.axis0.requested_state = (
                    AXIS_STATE_CLOSED_LOOP_CONTROL
                )
            else:
                raise Exception(
                    'Cannot set closed loop control for axis 0 if'
                    + ' it has not been calibrated'
                )
        if 1 in axes:
            if (
                self.odrive.axis1.motor.is_calibrated
                and self.odrive.axis1.encoder.is_ready
            ):
                self.odrive.axis1.requested_state = (
                    AXIS_STATE_CLOSED_LOOP_CONTROL
                )
            else:
                raise Exception(
                    'Cannot set closed loop control for axis 1 if'
                    + ' it has not been calibrated'
                )

    def set_ctrl_mode_torque(self, axes=[0, 1]):
        """
        Set the specified axes to torque control mode and subsequently set to
        closed loop control
        """
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = (
                CONTROL_MODE_TORQUE_CONTROL
            )
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = (
                CONTROL_MODE_TORQUE_CONTROL
            )
        self.set_axis_state_closed_loop(axes)

    def set_ctrl_mode_position(self, axes=[0, 1]):
        """
        Set the specified axes to position control mode and subsequently set to
        closed loop control
        """
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = (
                CONTROL_MODE_POSITION_CONTROL
            )
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = (
                CONTROL_MODE_POSITION_CONTROL
            )
        self.set_axis_state_closed_loop(axes)

    def set_ctrl_mode_velocity(self, axes=[0, 1]):
        """
        Set the specified axes to velocity control mode and subsequently set to
        closed loop control
        """
        if 0 in axes:
            self.odrive.axis0.controller.config.control_mode = (
                CONTROL_MODE_VELOCITY_CONTROL
            )
        if 1 in axes:
            self.odrive.axis1.controller.config.control_mode = (
                CONTROL_MODE_VELOCITY_CONTROL
            )
        self.set_axis_state_closed_loop(axes)

    def hold(self, axes=[0, 1]):
        """
        Changes control mode to position and makes motors hold current position
        """
        self.set_ctrl_mode_position(axes)
        if 0 in axes:
            self.odrive.axis0.controller.input_pos = (
                self.odrive.axis0.encoder.shadow_count
            )
        if 1 in axes:
            self.odrive.axis1.controller.input_pos = (
                self.odrive.axis1.encoder.shadow_count
            )

    def restart(self):
        """
        Reboots the associated odrive controller
        """
        self.odrive.reboot()

    def fwd_kinematics(self, theta0: float, theta1: float) -> tuple[float]:
        """
        Calculate the end effector position in cartesian coordinates from the
        given arm configuration
        """
        x = self.arm0.length * np.cos(theta0) + self.arm1.length * np.cos(
            theta1
        )
        y = self.arm0.length * np.sin(theta0) + self.arm1.length * np.sin(
            theta1
        )
        return x, y

    def inv_kinematics(self, x, y) -> np.ndarray:
        """
        Calcultes the elbow down configuration required to achieve the given
        end effector position. Returns a tuple joint angles in radians
        """
        eps = np.finfo(float).eps
        rsq = x ** 2 + y ** 2
        r = np.sqrt(rsq)
        l0 = self.arm0.length
        l1 = self.arm1.length

        alpha = np.arccos((rsq + (l0 ** 2 - l1 ** 2)) / (2 * l0 * r + eps))
        beta = np.arccos(((l0 ** 2 + l1 ** 2) - rsq) / (2 * l0 * l1))
        gamma = np.arctan2(y, x)

        #  alpha = np.arccos((rsq + (self.arm0.length**2 - self.arm1.length**2))
        #                    / (2 * self.arm0.length * np.sqrt(rsq) + eps))
        #  beta  = np.arccos((self.arm0.length**2 + self.arm1.length**2 - rsq)
        #                    / (2 * self.arm0.length * self.arm1.length))
        #  gamma = np.arctan2(y, x)

        theta0 = gamma - alpha
        theta1 = np.pi - beta

        theta1 += theta0

        thetas = np.asarray([theta0, theta1])

        return thetas

    def inv_kinematics_num(self, x, y, tol=0.005, maxdepth=20) -> np.ndarray:
        # goal position
        # tols - diff between prev guess and current guess

        if self.odrive is not None:
            # initial guess - use arm's current pos
            thetas_old = np.array(self.get_config())
        else:
            # initial guess - use arm's home config
            thetas_old = np.array((-np.pi / 4, np.pi / 2))

        thetas = thetas_old

        pd = np.array([x, y])
        diff = float('inf')

        depth = 0
        while depth < maxdepth and diff > tol:
            p = np.array(self.fwd_kinematics(*thetas_old))

            thetas = thetas_old + 0.5 * self.inv_jacobian(*thetas_old) @ (
                pd - p
            )

            diff = np.linalg.norm(thetas - thetas_old) / np.linalg.norm(
                thetas_old
            )

            thetas_old = thetas.copy()

            depth += 1

        #  print(f'depth: {depth} | diff: {diff}')

        thetas = np.mod(thetas + np.pi, 2 * np.pi) - np.pi

        return thetas

    def jacobian(self, theta0: float, theta1: float) -> np.ndarray:
        """
        Given two arm angles in radians, return the jacobian matrix
        """
        return np.array(
            [
                [
                    -self.arm0.length * np.sin(theta0),
                    -self.arm1.length * np.sin(theta1),
                ],
                [
                    self.arm0.length * np.cos(theta0),
                    self.arm1.length * np.cos(theta1),
                ],
            ]
        )

    def inv_jacobian(self, theta0: float, theta1: float) -> np.ndarray:
        """
        Given two arm angles in radians, return the inverse jacobian
        """
        return np.linalg.pinv(self.jacobian(theta0, theta1))

    def rad2count(self, theta: float, axis: int) -> int:
        """
        Convert radians to encoder counts
        """
        try:
            if axis == 0:
                return theta * self.arm0.cnt_per_rad + self.arm0.zero
            elif axis == 1:
                return theta * self.arm1.cnt_per_rad + self.arm1.zero
        except TypeError:
            print('Unable to convert radians to counts; check arm calibrated')
            raise

    def count2rad(self, count: int, axis: int) -> float:
        """
        Convert encoder counts to radians
        """
        try:
            if axis == 0:
                return (count - self.arm0.zero) / self.arm0.cnt_per_rad
            elif axis == 1:
                return (count - self.arm1.zero) / self.arm1.cnt_per_rad
        except TypeError:
            print('Unable to convert counts to radians; check arm calibrated')
            raise

    def get_config(self) -> tuple[float]:
        """
        Returns the current arm configuration as a tuple of joint angles
        in radians
        """
        count0 = self.odrive.axis0.encoder.shadow_count
        count1 = self.odrive.axis1.encoder.shadow_count

        theta0 = self.count2rad(count0, 0)
        theta1 = self.count2rad(count1, 1)

        return theta0, theta1


class ArmSegment:
    """
    Wrapper for data associated to each arm segment of the parent device.
    """

    def __init__(self, length=0.2):
        self.length = length
        self.zero = None
        self.cnt_per_rad = None
