import calculate
import device

import collections
import matplotlib.pyplot as plt
import seaborn as sb
import numpy as np
import time
import os
import sys

# check for OS and add sdl dlls if on windows
if sys.platform == 'win32':
    sdlpath = os.path.join(os.path.dirname(__file__), 'lib')
    os.environ['PYSDL2_DLL_PATH'] = sdlpath
# include resources
font = os.path.join(os.path.dirname(__file__), 'font/Inconsolata-Regular.ttf')

import sdl2
import sdl2.ext


# define some basic colors
WHITE  = sdl2.ext.Color(255,255,255)
BLACK  = sdl2.ext.Color(0,0,0)
RED    = sdl2.ext.Color(255,0,0)
GREEN  = sdl2.ext.Color(0,255,0)
BLUE   = sdl2.ext.Color(0,0,255)
ORANGE = sdl2.ext.Color(232,90,9)

# standard window size
win_width = 800
win_height = 600

# global elapsed time variable to keep track of visualization frame rate and
# simulation rate
elapsed_time = 0

# Render system to handle rendering texture sprites (the robot)
class TextureRenderSystem(sdl2.ext.TextureSpriteRenderSystem):
    """TextureRenderSystem is a class which converts texures to sdl sprites"""
    def __init__(self, renderer):
        super(TextureRenderSystem, self).__init__(renderer)
        self.renderer = renderer

# arm segment
class ArmSegment(sdl2.ext.Entity):
    """ArmSegment is a class which holds the necessary utility functions and
    graphics for a single arm segment (one rectangle in the visualization)"""
    def __init__(self, world, sprite, wposi=0, wposj=0, angle=0):
        pos = calculate.Coord(wpos=(wposi, wposj),
                               win_width=win_width,
                               win_height=win_height)
        self.sprite = sprite
        self.sprite.pos = pos
        self.sprite.angle = angle
        self.sprite.position = self.get_origin()
        self.sprite.point = sdl2.SDL_Point(int(self.sprite.size[0]/2), 0)

    def get_origin(self):
        """gets the top left corner of segment for sdl rect in window cords"""
        cornerx = int(self.sprite.pos.window.j - self.sprite.size[0]/2)
        cornery = self.sprite.pos.window.i
        return (cornerx, cornery)

    def get_end(self):
        """gets the middle of the end of segment in polar cords"""
        ox, oy = self.sprite.pos.cartesian
        x = ((self.sprite.size[1]/calculate.PIXELS_PER_METER)
              * np.cos(np.radians(-self.sprite.angle)) + ox)
        y = ((self.sprite.size[1]/calculate.PIXELS_PER_METER)
              * np.sin(np.radians(-self.sprite.angle)) + oy)
        end = calculate.Coord(cpos=(x,y), win_width=win_width,
                              win_height=win_height)
        return end.polar.r, end.polar.theta

    def get_config(self):
        """returns the equivalent joint angle of the arm segment"""
        return np.radians(-self.sprite.angle)

    def update(self, wposj, wposi, angle):
        """Sets the position and angle of the segment on the window to the
        given window coordinates and angle"""
        self.sprite.pos.window = (wposi, wposj)
        self.sprite.angle = angle

    def draw(self, spriterenderer):
        """Creates the sprite on the given renderer"""
        cornerx, cornery = self.get_origin()
        r = sdl2.SDL_Rect(cornerx, cornery, self.sprite.size[0],
                          self.sprite.size[1])
        p = self.sprite.point
        sdl2.SDL_RenderCopyEx(spriterenderer.sdlrenderer, self.sprite.texture,
                              None, r, self.sprite.angle, p, 0)

class SDLWrapper():
    """SDLWrapper holds together all the basic SDL systems and other classes
    which help in producing the graphics"""
    def __init__(self, window = None, renderer = None, fontmanager = None,
                 spritefactory = None, world = None, spriterenderer = None):
        self.window = window
        self.renderer = renderer
        self.fontmanager = fontmanager
        self.spritefactory = spritefactory
        self.world = world
        self.spriterenderer = spriterenderer
        self.vectors = None
        self.config_buffer = collections.deque([])

        # init sdl systems
        sdl2.ext.init()

        # create window, renderer, and the factory to create sprites
        if self.window is None:
            self.window = sdl2.ext.Window("Haptic Device",
                                          size=(win_width,win_height))
        if self.renderer is None:
            self.renderer = sdl2.ext.Renderer(self.window)
        if self.fontmanager is None:
            self.fontmanager = sdl2.ext.FontManager(font)
        if self.spritefactory is None:
            self.spritefactory = sdl2.ext.SpriteFactory(
                sdl2.ext.TEXTURE,
                renderer=self.renderer,
                fontmanager=self.fontmanager)

        if self.world is None:
            # create world
            self.world = sdl2.ext.World()

        if self.spriterenderer is None:
            # create sprite renderer with the texture system
            self.spriterenderer = TextureRenderSystem(self.renderer)

        # add sprite renderer to world
        self.world.add_system(self.spriterenderer)

        # make the window visible and start simulation loop
        self.window.show()

    def text(self, text: list, size=16):
        """Takes a list of lists and converts the contents into lines of text
        placed at the bottom left of the window"""
        start_height = win_height-size*len(text)
        start_width = 0
        text_sprites = []
        for i in range(len(text)):
            for j in range(len(text[i])):
                text_sprites.append(self.spritefactory.from_text(text[i][j],
                        size=size, color=BLACK, bg_color=WHITE))
                text_sprites[-1].position = (start_width, start_height)
                start_width += text_sprites[-1].size[0]+16
                if start_width > win_width:
                    print(i, j, text)
                    raise Exception('Line too long to fit on window')
            start_height += size
            start_width = 0
        self.text_sprites = text_sprites

    def vector_stream_plot(self, arm, vf):
        """Visualizes the given vector field with a stream plot. Small
        rectangles mark the sampling points and the trails move in the direction
        of the vector field at those sampled points. Green rect shows the center
        of the vector field."""
        center = calculate.Coord(cpos=(vf.args['xcenter'], vf.args['ycenter']),
                                 win_width=win_width, win_height=win_height)

        self.vectors = [] # stores points of vector lines
        self.squares = [] # stores sampled points
        self.center = [center.window.j-5, center.window.i-5, 10, 10]

        # step size defines how much space between each sample point in number
        # of pixels
        step_size = 40
        # iterate over the width and height of the window, includes bounds if
        # step_size is a multiple of height and width
        for y in np.arange(0, win_height//step_size*step_size+1, step_size):
            for x in np.arange(0, win_width//step_size*step_size+1, step_size):
                # add initial point to squares and start a vector line
                vector = []
                coord = calculate.Coord(wpos=(y, x), win_width=win_width,
                                        win_height=win_height)
                vector.extend((int(coord.window.j), int(coord.window.i)))
                self.squares.append((x-3, y-3, 6, 6))
                # sample n points in direction of vf from origin and add to the
                # list of vectors
                for _ in range(20):
                    dx, dy = vf.return_vectors(*coord.cartesian)
                    dx /= 50
                    dy /= 50
                    coord.cartesian = (coord.cartesian.x+dx,
                                       coord.cartesian.y+dy)
                    vector.extend((int(coord.window.j), int(coord.window.i)))
                self.vectors.append(vector)

    def theta_heatmap(self, arm, vf, axis):
        """
        Create and update a texture that is a single color channel heatmap
        representing the theta velocities.
        """
        # TODO heatmap for theta velocities
        # [x] calculate ik
        # [x] write method to impl newton-raphson (numerical ik)
        # [x] click on point in space and it calcs config and draws arm
        # [ ] for range of theta0 and theta1 calc jacobian and ratio of
        #     eigenvalues of jacobian, smaller eigenvalue / bigger eigenvalue

        #  print('analytical:', arm.inv_kinematics(0, 0.4))
        #  print('numerical:', arm.inv_kinematics_num(0, 0.4))

        binsz = 1
        num_samples = win_width*win_height // (binsz**2)

        I, J = np.meshgrid(np.arange(0, win_height, binsz),
                           np.arange(0, win_width, binsz))
        #  print(I.shape, J.shape)
        I = I.flatten()
        J = J.flatten()

        X, Y = calculate.Coord(wpos=(I,J)).cartesian

        #  XYsq = X**2 + Y**2
        #  lensq = arm.arm0.length**2 + arm.arm1.length**2 + 0.01**2
        #  X = np.delete(X, np.argwhere((XYsq > 0.01**2) & (XYsq < lensq)))
        #  Y = np.delete(Y, np.argwhere((XYsq > 0.01**2) & (XYsq < lensq)))

        I, J = calculate.Coord(cpos=(X,Y)).window

        dX, dY = vf.return_vectors(X, Y)

        if os.path.isfile('data.npz'):
            data = np.load('data.npz')
            dthetamatrix0 = data['dthetamatrix0']
            dthetamatrix1 = data['dthetamatrix1']
            Th0 = data['Th0']
            Th1 = data['Th1']
        else:
            Th0 = np.empty(num_samples)
            Th1 = np.empty(num_samples)
            for idx, (x, y) in enumerate(zip(X, Y)):
                theta0, theta1 = arm.inv_kinematics_num(x, y)
                Th0[idx] = theta0
                Th1[idx] = theta1
                print(f'Calculating IK: {idx+1}/{num_samples}', end='\r')
            print()

            dthetamatrix0 = np.empty(num_samples)
            dthetamatrix1 = np.empty(num_samples)
            for idx, (th0, th1, dx, dy) in enumerate(zip(Th0, Th1, dX, dY)):
                dthetas = arm.inv_jacobian(th0, th1) @ np.array([dx, dy])
                dthetamatrix0[idx] = dthetas[0]
                dthetamatrix1[idx] = dthetas[1]
                print(f'Calculating ijac: {idx+1}/{num_samples}', end='\r')
            print()

            np.savez_compressed('data', Th0=Th0, Th1=Th1,
                                dthetamatrix0=dthetamatrix0,
                                dthetamatrix1=dthetamatrix1)

        #  for row in dthetamatrix0.reshape((win_width//binsz, win_height//binsz)):
        #      print(row)

        plt.figure(figsize=(win_width/100, win_height/100))
        sb.heatmap(dthetamatrix0.reshape((win_width//binsz, win_height//binsz)),
                   vmin=-2, vmax=2)
                   #  cbar=False, xticklabels=False, yticklabels=False)
        plt.savefig('dtheta0.png')

        plt.figure(figsize=(win_width/100, win_height/100))
        sb.heatmap(Th0.reshape((win_width//binsz, win_height//binsz)),
                   vmin=-2*np.pi, vmax=2*np.pi)
                   #  cbar=False, xticklabels=False, yticklabels=False)
        plt.savefig('theta0.png')

        plt.figure(figsize=(win_width/100, win_height/100))
        sb.heatmap(dthetamatrix1.reshape((win_width//binsz, win_height//binsz)),
                   vmin=-2, vmax=2)
                   #  cbar=False, xticklabels=False, yticklabels=False)
        plt.savefig('dtheta1.png')

        plt.figure(figsize=(win_width/100, win_height/100))
        sb.heatmap(Th1.reshape((win_width//binsz, win_height//binsz)),
                   vmin=-2*np.pi, vmax=2*np.pi)
                   #  cbar=False, xticklabels=False, yticklabels=False)
        plt.savefig('theta1.png')

    def generate_device(self, arm=device.HapticDevice(init_with_device=False)):
        """Generate device takes the haptic device class and turns it into two
        arm segments that will represent the arm"""
        arm0_sprite = self.spritefactory.from_color(
                BLUE,
                size=(30,int(arm.arm0.length*calculate.PIXELS_PER_METER)))
        arm0 = ArmSegment(self.world, arm0_sprite, wposj=win_width/2,
                          wposi=0, angle=0)

        endr, endtheta = arm0.get_end()
        pos = calculate.Coord(ppos=(endr, endtheta), win_width=win_width,
                               win_height=win_height)
        arm1_sprite = self.spritefactory.from_color(
                ORANGE,
                size=(30,int(arm.arm1.length*calculate.PIXELS_PER_METER)))
        arm1 = ArmSegment(self.world, arm1_sprite, wposj=pos.window.j,
                          wposi=pos.window.i, angle=0)
        self.arms = (arm0, arm1)

    def update_device(self, theta0, theta1):
        """Updates the device configuration with the given angles"""
        pos = self.arms[0].sprite.pos
        angle0 = np.degrees(-theta0) # pi - theta0
        self.arms[0].update(pos.window.j, pos.window.i, angle0)

        r, theta = self.arms[0].get_end()
        pos = calculate.Coord(ppos=(r, theta), win_width=win_width,
                              win_height=win_height)
        angle1 = np.degrees(-theta1)
        self.arms[1].update(pos.window.j, pos.window.i, angle1)

    def smooth_move_to_location(self, arm, x, y, resolution=0.001):
        if (len(self.config_buffer) > 0):
            thetas = self.config_buffer[-1]
        else:
            thetas = [arm.get_config() for arm in self.arms]

        p = arm.fwd_kinematics(*thetas)
        pd = np.array([x, y])

        v = np.array([x, y]) - np.array(p)
        vhat = v / np.linalg.norm(v)

        num_ik = arm.inv_kinematics_num(*pd)
        an_ik = arm.inv_kinematics(*pd)
        print(f'num ik: {num_ik} | ana ik: {an_ik}')

        while (np.linalg.norm(pd - (p := resolution*vhat + p)) > resolution):
            thetas = arm.inv_kinematics_num(*p)
            self.config_buffer.append(thetas)

    # run simulation
    def step(self):
        """Advances the visualization one time/sim step ahead"""
        # time frame start
        frame_start = time.monotonic()

        # clear old graphics and update state for next frame
        self.renderer.clear()

        if self.vectors is not None:
            for vector in self.vectors:
                self.renderer.draw_line(vector, color=WHITE)
            self.renderer.draw_rect(self.squares, color=WHITE)
            self.renderer.fill(self.center, color=GREEN)

        # render arm sprites
        for arm in self.arms:
            arm.draw(self.spriterenderer)
        # render text sprites onto window
        if self.text_sprites:
            self.spriterenderer.render(self.text_sprites)

        self.renderer.present()

        global elapsed_time
        # preserve physics by calculating elapsed time and delaying if necessary
        # to limit to only 60 FPS. This prevents the robot velocity being
        # dependent on the frame rate of the simulation
        elapsed_time = time.monotonic() - frame_start
        if elapsed_time < 1/60:
            sdl2.SDL_Delay(int((1/60*1000)-(elapsed_time*1000)))
            elapsed_time = time.monotonic() - frame_start

def main():
    # init visualization stuff
    vis = SDLWrapper()
    vis.generate_device()

    arm = device.HapticDevice(False)

    # take cli args for vf
    if '-field' in sys.argv:
        field = sys.argv[sys.argv.index('-field')+1]
        if field not in calculate.VectorField(arm)._fields.keys():
            raise UserWarning('Supplied field type does not exist')
    else:
        field = 'circle'

    vf_args = {
            'xcenter': np.sqrt(2)*arm.arm0.length,
            'ycenter': 0,
            'dtheta' : .5,
            'radius' : arm.arm0.length/2,
            'buffer' : arm.arm0.length/4 * .05,
            'drmax'  : .5,
            }
    vf = calculate.VectorField(arm, field=field, args=vf_args)

    vis.vector_stream_plot(arm, vf)

    if '-hm' in sys.argv:
        vis.theta_heatmap(arm, vf, 0)

    vis.update_device(0, np.pi/2)

    x, y = 0, 0
    i, j = 0, 0
    recalculate = False

    running = True
    while running:
        t = time.monotonic()

        #  sdl2.SDL_UpdateTexture(tex.texture, None, pixels, win_width*4)

        if '-animate' in sys.argv:
            theta0 = np.pi/4*np.sin(t)
            theta1 = np.pi/4*np.cos(2*t)
            vis.update_device(theta0, theta1)
        elif '-follow' in sys.argv:
            if recalculate:
                print(f'xy: ({x}, {y}) | ij: ({i}, {j})')
                vis.smooth_move_to_location(arm, x, y)
                recalculate = False
                if (len(vis.config_buffer) > 0):
                    vis.update_device(*vis.config_buffer.popleft())
            elif (len(vis.config_buffer) > 0):
                vis.update_device(*vis.config_buffer.popleft())
        else:
            theta0, theta1 = 0, np.pi/2
            vis.update_device(theta0, theta1)

        text = [[str(vis.arms[0].get_end()[0])[:5],
                 str(vis.arms[0].get_end()[1])[:5]],
                [str(vis.arms[1].sprite.pos.window.j)[:5],
                 str(vis.arms[1].sprite.pos.window.i)[:5]]]
        vis.text(text)

        vis.step()
        # check for window being closed or keypresses and keyboard control
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                sdl2.ext.quit()
                running = False
                break
            if event.type == sdl2.SDL_MOUSEMOTION:
                j, i = event.motion.x, event.motion.y
            if event.type == sdl2.SDL_MOUSEBUTTONDOWN:
                if event.button.button == sdl2.SDL_BUTTON_LEFT:
                    x, y = calculate.Coord(wpos=(i, j)).cartesian
                    recalculate = True

if __name__ == "__main__":
    sys.exit(main())
