import numpy as np
import time
import ctypes
from sdl2 import *

WIN_WIDTH = 800
WIN_HEIGHT = 600

class prev_key_state_t():
    def __init__(self):
        self.key = {}

class polar_pos_t():
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

class mouse_state_t(win_pos_t):
    def __init__(self, win_pos = win_pos_t()):
        self.left_button = False
        self.right_button = False
        self.win_pos_t = win_pos

def update_mouse_state():
    result = mouse_state_t()
    PI = POINTER(c_int)
    mouse_button_state = SDL_GetMouseState(PI(c_int(result.win_pos_t.x)), PI(c_int(result.win_pos_t.y)))
    result.left_button = not ((mouse_button_state & SDL_BUTTON(SDL_BUTTON_LEFT)) == 0)
    result.right_button = not ((mouse_button_state & SDL_BUTTON(SDL_BUTTON_RIGHT)) == 0)
    return result

class color_t():
    def __init__(self, r = 0, g = 0, b = 0):
        self.r = r
        self.g = g
        self.b = b

class arm_segment_t():
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
        pixels = np.empty(self.width*self.length*4)
        for i in range(self.width*self.length):
            p = i*4
            pixels[p] = self.color_t.r
            pixels[p+1] = self.color_t.g
            pixels[p+2] = self.color_t.b
        self.tex = pixels_to_texture(renderer, pixels, self.width, self.length)
    def draw(self, renderer):
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
        cpos = self.ppos.to_cart()
        ox, oy = cpos.x, cpos.y
        x = np.sin(deg_to_rad(self.rot))*self.length + ox
        y = np.cos(deg_to_rad(self.rot))*self.length + oy
        cpos_end = cart_pos_t(x, y)
        return cpos_end.to_polar()

field_t = {
        'CIRCLE': 0,
        'CIRCLEBOUND': 1,
        'SPIRAL': 2,
        'SPIRALBOUND': 3
        }

def usr_input():
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

class vector_field_t():
    def __init__(self, center = cart_pos_t(0,0), desc = {}, kind = 0):
        self.center = center
        self.desc = desc
        self.kind = kind
    def define(self, arm0, arm1, selection):
        self.kind = selection
        self.desc = {}
        if selection == field_t['CIRCLE']:
            self.center = cart_pos_t(arm1.length, arm0.length)
            self.desc["dtheta"] = np.pi / 2
        elif selection == field_t['CIRCLEBOUND']:
            self.center = cart_pos_t(arm1.length, arm0.length)
            self.desc["dr"] = arm1.length / 8
            self.desc["radius"] = arm1.length / 2
            self.desc["buffer"] = arm1.length / 20
        elif selection == field_t['SPIRAL']:
            self.center = cart_pos_t(arm1.length, arm0.length)
            self.desc["dtheta"] = np.pi / 2
            self.desc["dr"] = arm1.length / 8
        elif selection == field_t['SPIRALBOUND']:
            self.center = cart_pos_t(arm1.length, arm0.length)
            self.desc["dtheta"] = np.pi / 2
            self.desc["dr"] = arm1.length / 8
            self.desc["radius"] = arm1.length / 2
            self.desc["buffer"] = arm1.length / 20
    def update_arm(self, arm0, arm1, elapsed_time):
        selection = self.kind
        if selection == field_t['CIRCLE']:
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
        elif selection == field_t['CIRCLEBOUND']:
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
        elif selection == field_t['SPIRAL']:
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
        elif selection == field_t['SPIRALBOUND']:
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
    def get_arrow_rot(self, cpos):
        selection = self.kind
        if selection == field_t['CIRCLE']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, 0, self.desc["dtheta"])
            return dx, dy
        elif selection == field_t['CIRCLEBOUND']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dr = self.desc["dr"]
            if ppos.r > self.desc["radius"]+self.desc["buffer"]:
                dr *= 1
            elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
                dr *= -1
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, 0)
            return dx, dy
        elif selection == field_t['SPIRAL']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, self.desc["dr"], self.desc["dtheta"])
            return dx, dy
        elif selection == field_t['SPIRALBOUND']:
            cpos = shift_origin(cpos, self.center)
            ppos = cpos.to_polar()
            dr = self.desc["dr"]
            if ppos.r > self.desc["radius"]+self.desc["buffer"]:
                dr *= 1
            elif ppos.r < self.desc["radius"]-self.desc["buffer"]:
                dr *= -1
            dx, dy = polar_to_cart_d(ppos.r, ppos.theta, dr, self.desc["dtheta"])
            return dx, dy
        return numpy.finfo(float).max, numpy.finfo(float).max

def shift_origin(cpos, new_origin):
    return cart_pos_t(cpos.x - new_origin.x, cpos.y - new_origin.y)

def deg_to_rad(deg):
    return deg * np.pi / 180

def rad_to_deg(rad):
    return -(rad * 180 / np.pi)

def fwd_kinematics(arm0, arm1):
    x = arm0.length*np.cos(deg_to_rad(arm0.rot)) + arm1.length*np.cos(deg_to_rad(arm1.rot))
    y = arm0.length*np.sin(deg_to_rad(arm0.rot)) + arm1.length*np.sin(deg_to_rad(arm1.rot))
    return cart_pos_t(x, y)

def jacobian(arm0, arm1):
    j = np.matrix(
            [[-arm0.length*np.sin(deg_to_rad(arm0.rot)), -arm1.length*np.sin(deg_to_rad(arm1.rot))],
            [arm0.length*np.cos(deg_to_rad(arm0.rot)), arm1.length*np.cos(deg_to_rad(arm1.rot))]]
            )
    return j

def inv_jacobian(arm0, arm1):
    j = jacobian(arm0, arm1)
    jinv = np.linalg.pinv(j)
    return jinv

def polar_to_cart_d(r, theta, dr, dtheta):
    dx = dr*np.cos(theta) - r*np.sin(theta)*dtheta
    dy = dr*np.sin(theta) + r*np.cos(theta)*dtheta
    return dx, dy

class screen_state_t():
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
    index = (y*WIN_WIDTH + x) * 4
    if index < len(pixels)-4 and index >= 0:
        pixels[index] = c.r
        pixels[index+1] = c.g
        pixels[index+2] = c.b

def pixels_to_texture(renderer, pixels, width, height):
    tex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, width, height)
    SDL_UpdateTexture(tex, None, bytes(pixels), width*4)
    return tex

def update_mouse(arm0, arm1, key_state, mouse_state, elapsed_time):
    if key_state[SDL_SCANCODE_A] != 0:
        arm0.rot -= 1
    if key_state[SDL_SCANCODE_D] != 0:
        arm0.rot += 1
    arm0_end = arm0.get_end()
    mouse_cart_adj = shift_origin(mouse_state.win_pos_t.to_cart(), arm0_end.to_cart())
    mouse_polar_adj = mouse_cart_adj.to_polar()
    arm1.ppos = arm0_end
    arm1.rot = rad_to_deg(mouse_polar_adj.theta) + 90
    
def main():
    cpos0 = cart_pos_t(0, 10)
    ppos0 = cpos0.to_polar()
    arm0 = arm_segment_t(tex = None, width = 20, length = 220, ppos = ppos0, rot = 0, color_t = color_t(0,255,0), num = 0)
    ppos1 = arm0.get_end()
    arm1 = arm_segment_t(tex = None, width = 20, length = 220, ppos = ppos1, rot = 110, color_t = color_t(255,0,0), num = 1)
    screen_state = screen_state_t(show_center = False, show_field = False, show_trace = False, interactive = False)

    vf = vector_field_t()
    vf.define(arm0, arm1, usr_input())
    
    SDL_Init(SDL_INIT_EVERYTHING)
    window = SDL_CreateWindow(b"SCARA Arm", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN)
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED)
    
    arm0.generate(renderer)
    arm1.generate(renderer)

    square_pixels = np.empty(10*10*4)
    for i in range(10*10):
        p = i * 4
        square_pixels[p+2] = 255
    square_tex = pixels_to_texture(renderer, square_pixels, 10, 10)
    square_rect = SDL_Rect(x = int(vf.center.to_win().x - 5), y = int(vf.center.to_win().y - 15), w = 10, h = 10)

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
    arrow_tex = pixels_to_texture(renderer, arrow_pixels, 3, 5)

    mouse_state = update_mouse_state()
    key_state = SDL_GetKeyboardState(None)
    key_state_prev = prev_key_state_t()
    key_state_prev.key["c"] = False
    key_state_prev.key["v"] = False
    key_state_prev.key["t"] = False
    key_state_prev.key["0"] = False
    key_state_prev.key["1"] = False
    key_state_prev.key["2"] = False
    key_state_prev.key["3"] = False

    pixels = np.empty(WIN_WIDTH*WIN_HEIGHT*4)
    bg_tex = pixels_to_texture(renderer, pixels, WIN_WIDTH, WIN_HEIGHT)

    traces = []
    trace_idx = 0
    trace_rect = SDL_Rect(x = -10, y = -10, w = 10, h = 10)
    for i in range(600):
        traces.append(trace_rect)

    frame_start = time.monotonic()

    event = SDL_Event()

    elapsed_time = 0.001
    while True:
        frame_start = time.monotonic()
        mouse_state = update_mouse_state()
        while SDL_PollEvent(ctypes.byref(event)) != 0:
            if event.type == SDL_QUIT:
                SDL_DestroyRenderer(renderer)
                SDL_DestroyWindow(window)
                SDL_Quit()
                return

        if screen_state.interactive:
            update_mouse(arm0, arm1, key_state, mouse_state, elapsed_time)
        else:
            vf.update_arm(arm0, arm1, elapsed_time)

        if elapsed_time != 0:
            fps = 1/elapsed_time
        else:
            fps = 999999.99
        print(
            "Window: %3.0f %3.0f | Cartesian: %4.0f %3.0f | Polar: %7.3f %5.3f | FPS: %6.2f | MSPF: %5.2f\n"
            %(mouse_state.win_pos_t.x, mouse_state.win_pos_t.y,
            mouse_state.win_pos_t.to_cart().x, mouse_state.win_pos_t.to_cart().y,
            mouse_state.win_pos_t.to_polar().r, mouse_state.win_pos_t.to_polar().theta,
            fps, elapsed_time*1000.0)
        )

        bg_update(renderer, vf, arm0, arm1, screen_state, key_state, key_state_prev, pixels)
        SDL_RenderCopy(renderer, bg_tex, None, None)
        if screen_state.show_center:
            SDL_RenderCopy(renderer, square_tex, None, square_rect)
        if screen_state.show_trace:
            ppos = arm1.get_end()
            wpos = ppos.to_win()
            trace_rect = SDL_Rect(x = int(wpos.x), y = int(wpos.y), w = 5, h = 5)
            traces[trace_idx % len(traces)] = trace_rect
            trace_idx += 1
            for i in range(len(traces)):
                SDL_RenderCopy(renderer, square_tex, None, traces[i])
        if screen_state.show_field:
            for y in range(int(WIN_HEIGHT/16)):
                for x in range(int(WIN_HEIGHT/12)):
                    wpos = win_pos_t(x*50, y*50)
                    dx, dy = vf.get_arrow_rot(wpos.to_cart())
                    theta = rad_to_deg(np.arctan2(dy, dx)) + 90
                    arrow_rect = SDL_Rect(x = x*50, y = y*50, w = 6, h = 10)
                    SDL_RenderCopyEx(renderer, arrow_tex, None, arrow_rect, theta, None, 0)

        arm0.draw(renderer)
        arm1.draw(renderer)
        SDL_RenderPresent(renderer)

        elapsed_time = time.monotonic() - frame_start
        if elapsed_time < .005:
            SDL_Delay(int(5-elapsed_time*1000))
            elapsed_time = time.monotonic() - frame_start

main()
