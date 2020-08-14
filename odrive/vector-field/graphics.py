import numpy as np
import time
import ctypes
import configparser
from calculate import *
from sdl2 import *

config = configparser.ConfigParser()
config.read('config.ini')

WIN_WIDTH = int(config['window']['width'])
WIN_HEIGHT = int(config['window']['height'])

class sim_t():
    def __init__(self):
        pass

class prev_key_state_t():
    '''
    Prev key state keeps track of the which keys were pressed each frame
    '''
    def __init__(self):
        self.key = {}

class mouse_state_t(win_pos_t):
    '''
    Mouse state keeps track of mouse clicks and cursor position
    '''
    def __init__(self, win_pos = win_pos_t()):
        self.left_button = False
        self.right_button = False
        self.win_pos_t = win_pos
    def update_mouse_state(self):
        '''
        Returns a new mouse state with most recent information
        '''
        PI = POINTER(c_int)
        mouse_button_state = SDL_GetMouseState(
                PI(c_int(self.win_pos_t.x)), PI(c_int(self.win_pos_t.y)))
        self.left_button = not ((mouse_button_state & SDL_BUTTON(SDL_BUTTON_LEFT)) == 0)
        self.right_button = not ((mouse_button_state & SDL_BUTTON(SDL_BUTTON_RIGHT)) == 0)

class color_t():
    '''
    Type to hold pixel color information
    '''
    def __init__(self, r = 0, g = 0, b = 0, a = 0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

class arm_segment_t():
    '''
    Stores texture and arm segment size as well as helper functions for
    handling the segment.
    '''
    def __init__(
            self,
            tex,
            width = 0,
            length = 0,
            ppos = polar_pos_t(0,0),
            rot = 0,
            color_t = color_t(0,0,0),
            num = 0
            ):
        self.tex = tex
        self.width = width
        self.length = length
        self.ppos = ppos
        self.rot = rot
        self.color_t = color_t
        self.num = num
    def generate(self, renderer):
        '''
        Creates a texture for the arm segment according to defined size
        '''
        pixels = np.empty(self.width*self.length*4)
        for i in range(self.width*self.length):
            p = i*4
            pixels[p] = self.color_t.r
            pixels[p+1] = self.color_t.g
            pixels[p+2] = self.color_t.b
        self.tex = pixels_to_texture(renderer, pixels, self.width, self.length)
    def draw(self, renderer):
        '''
        Passes the texture to the renderer to draw pixels on the screen
        '''
        wpos = self.ppos.to_win()
        x, y = wpos.x, wpos.y
        corner_x = x - self.width/2
        corner_y = y - self.length
        rect = SDL_Rect(
                x = int(corner_x),
                y = int(corner_y),
                w = int(self.width),
                h = int(self.length)
                )
        point = SDL_Point(x = int(self.width/2), y = int(self.length))
        SDL_RenderCopyEx(renderer, self.tex, None, rect, self.rot, point, 0)
    def get_end(self):
        '''
        Gets the center point of the edge farthest from the polar origin
        '''
        cpos = self.ppos.to_cart()
        ox, oy = cpos.x, cpos.y
        x = np.sin(deg_to_rad(self.rot))*self.length + ox
        y = np.cos(deg_to_rad(self.rot))*self.length + oy
        cpos_end = cart_pos_t(x, y)
        return cpos_end.to_polar()

def usr_input():
    '''
    Grab user input for vector field type
    '''
    print("Possible field types:")
    print("\t(0) Circle\n\t(1) Circle Bound\n\t(2) Spiral\n\t(3) Spiral Bound")
    try:
        selection = int(input("Selection: "))
        if selection > 3 or selection < 0:
            raise ValueError('Selection out of bounds')
        return selection
    except ValueError:
        raise
    except:
        raise

class screen_state_t():
    '''
    Keeps track of use selected parameters
    '''
    def __init__(
            self,
            show_center = False,
            show_field = False,
            show_trace = False,
            interactive = False,
            circle = False,
            circle_bound = False,
            spiral = False,
            spiral_bound = False
            ):
        self.show_center = show_center
        self.show_field = show_field
        self.show_trace = show_trace
        self.interactive = interactive
        self.circle = circle
        self.circle_bound = circle_bound
        self.spiral = spiral
        self.spiral_bound = spiral_bound

def bg_update(renderer, vf, arm0, arm1, screen_state, key_state, key_state_prev, pixels):
    '''
    update screen state to be used for drawing background texture
    '''
    if key_state[SDL_SCANCODE_C] != 0:
        if not key_state_prev.key["c"]:
            screen_state.show_center = not screen_state.show_center
        key_state_prev.key["c"] = True
    else:
        key_state_prev.key["c"] = False
    if key_state[SDL_SCANCODE_V] != 0:
        if not key_state_prev.key["v"]:
            screen_state.show_field = not screen_state.show_field
        key_state_prev.key["v"] = True
    else:
        key_state_prev.key["v"] = False
    if key_state[SDL_SCANCODE_T] != 0:
        if not key_state_prev.key["t"]:
            screen_state.show_trace = not screen_state.show_trace
        key_state_prev.key["t"] = True
    else:
        key_state_prev.key["t"] = False
    if key_state[SDL_SCANCODE_I] != 0:
        if not key_state_prev.key["i"]:
            screen_state.interactive = not screen_state.interactive
        key_state_prev.key["i"] = True
    else:
        key_state_prev.key["i"] = False
    if key_state[SDL_SCANCODE_0] != 0:
        if not key_state_prev.key["0"]:
            screen_state.circle = not screen_state.circle
            if screen_state.circle:
                vf.define(arm0, arm1, field_t["CIRCLE"])
        key_state_prev.key["0"] = True
    else:
        key_state_prev.key["0"] = False
    if key_state[SDL_SCANCODE_1] != 0:
        if not key_state_prev.key["1"]:
            screen_state.circle_bound = not screen_state.circle_bound
            if screen_state.circle_bound:
                vf.define(arm0, arm1, field_t["CIRCLEBOUND"])
        key_state_prev.key["1"] = True
    else:
        key_state_prev.key["1"] = False
    if key_state[SDL_SCANCODE_2] != 0:
        if not key_state_prev.key["2"]:
            screen_state.spiral = not screen_state.spiral
            if screen_state.spiral:
                vf.define(arm0, arm1, field_t["SPIRAL"])
        key_state_prev.key["2"] = True
    else:
        key_state_prev.key["2"] = False
    if key_state[SDL_SCANCODE_3] != 0:
        if not key_state_prev.key["3"]:
            screen_state.spiral_bound = not screen_state.spiral_bound
            if screen_state.spiral_bound:
                vf.define(arm0, arm1, field_t["SPIRALBOUND"])
        key_state_prev.key["3"] = True
    else:
        key_state_prev.key["3"] = False

def set_pixel(x, y, c, pixels):
    '''
    Set individual pixel in pixel array to given color
    '''
    index = (y*WIN_WIDTH + x) * 4
    if index < len(pixels)-4 and index >= 0:
        pixels[index] = c.r
        pixels[index+1] = c.g
        pixels[index+2] = c.b

def pixels_to_texture(renderer, pixels, width, height):
    '''
    Convert pixels to an sdl texture
    '''
    tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, width, height)
    SDL_UpdateTexture(tex, None, bytes(pixels), width*4)
    return tex

def update_mouse(arm0, arm1, key_state, mouse_state, elapsed_time):
    '''
    handle user input for moving arm
    '''
    if key_state[SDL_SCANCODE_A] != 0:
        arm0.rot -= 1
    if key_state[SDL_SCANCODE_D] != 0:
        arm0.rot += 1
    arm0_end = arm0.get_end()
    mouse_cart_adj = shift_origin(mouse_state.win_pos_t.to_cart(), arm0_end.to_cart())
    mouse_polar_adj = mouse_cart_adj.to_polar()
    arm1.ppos = arm0_end
    arm1.rot = rad_to_deg(mouse_polar_adj.theta) + 90

def init(arm):
    '''
    initialize classes and textures for running graphics
    '''
    # init sim class
    sim = sim_t()

    # generate two arm segments and place at default location
    cpos0 = cart_pos_t(0, 10)
    ppos0 = cpos0.to_polar()
    arm.gfx_seg0 = arm_segment_t(
            tex = None, 
            width = arm.width0, 
            length = arm.length0, 
            ppos = ppos0, 
            rot = 0, 
            color_t = color_t(0,255,0), 
            num = 0)
    ppos1 = arm.gfx_seg0.get_end()
    arm.gfx_seg1 = arm_segment_t(
            tex = None, 
            width = arm.width1, 
            length = arm.length1, 
            ppos = ppos1, 
            rot = 110, 
            color_t = color_t(255,0,0),
            num = 1)
    sim.screen_state = screen_state_t(
            show_center = False, 
            show_field = False, 
            show_trace = False, 
            interactive = False)

    # initialize sdl utility and create the window and renderer
    SDL_Init(SDL_INIT_EVERYTHING)
    sim.window = SDL_CreateWindow(b"SCARA Arm", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN)
    sim.renderer = SDL_CreateRenderer(sim.window, -1, SDL_RENDERER_ACCELERATED)

    # init two arm segments
    arm.gfx_seg0.generate(sim.renderer)
    arm.gfx_seg1.generate(sim.renderer)

    # create square to use as center of vector field marker
    square_pixels = np.empty(10*10*4)
    for i in range(10*10):
        p = i * 4
        square_pixels[p+2] = 255
    sim.square_tex = pixels_to_texture(sim.renderer, square_pixels, 10, 10)
    sim.square_rect = SDL_Rect(
            x = int(arm.vf.center.to_win().x - 5),
            y = int(arm.vf.center.to_win().y - 15), w = 10, h = 10)

    # create arrow textures to show vector field
    arrow_mask = [
            0,1,0,
            1,1,1,
            0,1,0,
            0,1,0,
            0,1,0
            ]
    arrow_pixels = np.empty(5*3*4)
    for i in range(5*3):
        p = i * 4
        if arrow_mask[i] > 0:
            arrow_pixels[p] = 255
            arrow_pixels[p+1] = 255
            arrow_pixels[p+2] = 255
    sim.arrow_tex = pixels_to_texture(sim.renderer, arrow_pixels, 3, 5)

    # initialize mouse state and keyboard state
    sim.mouse_state = mouse_state_t()
    sim.key_state = SDL_GetKeyboardState(None)
    sim.key_state_prev = prev_key_state_t()
    sim.key_state_prev.key["c"] = False
    sim.key_state_prev.key["v"] = False
    sim.key_state_prev.key["t"] = False
    sim.key_state_prev.key["0"] = False
    sim.key_state_prev.key["1"] = False
    sim.key_state_prev.key["2"] = False
    sim.key_state_prev.key["3"] = False

    # create texture for background of simulation window
    sim.pixels = np.empty(WIN_WIDTH*WIN_HEIGHT*4)
    sim.bg_tex = pixels_to_texture(sim.renderer, sim.pixels, WIN_WIDTH, WIN_HEIGHT)

    # create pixels to use as traces for arm end
    sim.traces = []
    sim.trace_idx = 0
    trace_rect = SDL_Rect(x = -10, y = -10, w = 10, h = 10)
    for i in range(600):
        sim.traces.append(trace_rect)

    # starting time
    sim.frame_start = time.monotonic()

    # create empty event to check for window exit later
    sim.event = SDL_Event()

    # prevent initial divide by zero errors
    sim.elapsed_time = 0.001

    return sim

def step(arm, sim):
    # update frame start for elapsed time
    sim.frame_start = time.monotonic()
    sim.mouse_state.update_mouse_state()

    # check for window exit
    while SDL_PollEvent(ctypes.byref(sim.event)) != 0:
        if sim.event.type == SDL_QUIT:
            SDL_DestroyRenderer(sim.renderer)
            SDL_DestroyWindow(sim.window)
            SDL_Quit()
            return

    # update arm via user control if enabled, otherwise use vector field
    if sim.screen_state.interactive:
        update_mouse(
                arm.gfx_seg0, 
                arm.gfx_seg1,
                sim.key_state,
                sim.mouse_state,
                sim.elapsed_time)
    else:
        arm.vf.update_arm(arm.gfx_seg0, arm.gfx_seg1, sim.elapsed_time)

    # print debug info to console
    if sim.elapsed_time != 0:
        fps = 1/sim.elapsed_time
    else:
        fps = 999999.99
    print(
        "Window: %3.0f %3.0f | Cartesian: %4.0f %3.0f | Polar: %7.3f %5.3f | FPS: %6.2f | MSPF: %5.2f\n"
        %(sim.mouse_state.win_pos_t.x, sim.mouse_state.win_pos_t.y,
        sim.mouse_state.win_pos_t.to_cart().x, sim.mouse_state.win_pos_t.to_cart().y,
        sim.mouse_state.win_pos_t.to_polar().r, sim.mouse_state.win_pos_t.to_polar().theta,
        fps, sim.elapsed_time*1000.0)
    )

    # update the background and draw textures
    bg_update(
            sim.renderer, 
            arm.vf, 
            arm.gfx_seg0, 
            arm.gfx_seg1, 
            sim.screen_state, 
            sim.key_state, 
            sim.key_state_prev, 
            sim.pixels)
    SDL_RenderCopy(sim.renderer, sim.bg_tex, None, None)
    if sim.screen_state.show_center:
        SDL_RenderCopy(sim.renderer, sim.square_tex, None, sim.square_rect)
    if sim.screen_state.show_trace:
        ppos = arm.gfx_seg1.get_end()
        wpos = ppos.to_win()
        trace_rect = SDL_Rect(x = int(wpos.x), y = int(wpos.y), w = 5, h = 5)
        sim.traces[sim.trace_idx % len(sim.traces)] = trace_rect
        sim.trace_idx += 1
        for i in range(len(sim.traces)):
            SDL_RenderCopy(sim.renderer, sim.square_tex, None, sim.traces[i])
    if sim.screen_state.show_field:
        for y in range(int(WIN_HEIGHT/16)):
            for x in range(int(WIN_HEIGHT/12)):
                wpos = win_pos_t(x*50, y*50)
                dx, dy = arm.vf.get_arrow_rot(wpos.to_cart())
                theta = rad_to_deg(np.arctan2(dy, dx)) + 90
                arrow_rect = SDL_Rect(x = x*50, y = y*50, w = 6, h = 10)
                SDL_RenderCopyEx(sim.renderer, sim.arrow_tex, None, arrow_rect, theta, None, 0)

    # draw arm over background
    arm.gfx_seg0.draw(sim.renderer)
    arm.gfx_seg1.draw(sim.renderer)

    # show newly drawn frame
    SDL_RenderPresent(sim.renderer)

    # update elapsed time and delay if too fast, avoid any physics issues
    sim.elapsed_time = time.monotonic() - sim.frame_start
    if sim.elapsed_time < .005:
        SDL_Delay(int(5-sim.elapsed_time*1000))
        sim.elapsed_time = time.monotonic() - sim.frame_start
