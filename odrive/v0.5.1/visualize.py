import calculate
import numpy as np
import time
import os
import sys

# TODO coordinate system, vector field, center point

if sys.platform == 'win32':
    sdlpath = os.path.join(os.path.dirname(__file__), 'lib')
    os.environ['PYSDL2_DLL_PATH'] = sdlpath
font = os.path.join(os.path.dirname(__file__), 'font/Inconsolata-Regular.ttf')

import sdl2
import sdl2.ext

WHITE  = sdl2.ext.Color(255,255,255)
BLACK  = sdl2.ext.Color(0,0,0)
RED    = sdl2.ext.Color(255,0,0)
GREEN  = sdl2.ext.Color(0,255,0)
BLUE   = sdl2.ext.Color(0,0,255)
ORANGE = sdl2.ext.Color(232,90,9)

win_width = 800
win_height = 600

elapsed_time = 0

# Render system to handle rendering texture sprites (the robot)
class TextureRenderSystem(sdl2.ext.TextureSpriteRenderSystem):
    def __init__(self, renderer):
        super(TextureRenderSystem, self).__init__(renderer)
        self.renderer = renderer

# arm segment
class ArmSegment(sdl2.ext.Entity):
    def __init__(self, world, sprite, wposx=0, wposy=0, angle=0):
        wpos = calculate.WinPos(wposx, wposy, win_width, win_height)
        self.sprite = sprite
        self.sprite.ppos = wpos.to_polar()
        self.sprite.angle = angle
        self.sprite.position = self.get_origin()
        self.sprite.point = sdl2.SDL_Point(int(self.sprite.size[0]/2), 0)

    def get_origin(self):
        '''gets the top left corner of segment for sdl rect in window cords'''
        wpos = self.sprite.ppos.to_win()
        x, y = wpos.x, wpos.y
        cornerx = int(x - self.sprite.size[0]/2)
        cornery = int(y)
        return (cornerx, cornery)

    def get_end(self):
        '''gets the middle of the end of segment in polar cords'''
        cpos = self.sprite.ppos.to_cart()
        ox, oy = cpos.x, cpos.y
        x = self.sprite.size[1]*np.cos(np.radians(-self.sprite.angle)) + ox
        y = self.sprite.size[1]*np.sin(np.radians(-self.sprite.angle)) + oy
        ppos_end = calculate.CartPos(x, y, win_width, win_height).to_polar()
        #  ppos = self.sprite.ppos
        #  r = ppos.r + self.sprite.size[1]
        #  theta = ppos.theta
        #  ppos_end = calculate.PolarPos(r, theta, win_width, win_height)
        return ppos_end.r, ppos_end.theta
    def update(self, wposx, wposy, angle):
        wpos = calculate.WinPos(wposx, wposy, win_width, win_height)
        self.sprite.ppos = wpos.to_polar()
        self.sprite.angle = angle

    def draw(self, spriterenderer):
        #  wpos = self.sprite.ppos.to_win()
        #  x, y = wpos.x, wpos.y
        cornerx, cornery = self.get_origin()
        r = sdl2.SDL_Rect(cornerx,
                          cornery,
                          self.sprite.size[0],
                          self.sprite.size[1])
        p = self.sprite.point
        sdl2.SDL_RenderCopyEx(spriterenderer.sdlrenderer,
                              self.sprite.texture,
                              None,
                              r,
                              self.sprite.angle,
                              p,
                              0)

class SDLWrapper():
    def __init__(self,
                 window = None,
                 renderer = None,
                 fontmanager = None,
                 spritefactory = None,
                 world = None,
                 spriterenderer = None):
        self.window = window
        self.renderer = renderer
        self.fontmanager = fontmanager
        self.spritefactory = spritefactory
        self.world = world
        self.spriterenderer = spriterenderer

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
        start_height = win_height-size*len(text)
        start_width = 0
        text_sprites = []
        for i in range(len(text)):
            for j in range(len(text[i])):
                text_sprites.append(self.spritefactory.from_text(text[i][j],
                                                                 size=size))
                text_sprites[-1].position = (start_width, start_height)
                start_width += text_sprites[-1].size[0]+16
                if start_width > win_width:
                    print(i, j, text)
                    raise Exception('Line too long to fit on window')
            start_height += size
            start_width = 0
        self.text_sprites = text_sprites

    def generate_device(self):
        arm0_sprite = self.spritefactory.from_color(BLUE, size=(30,200))
        arm0 = ArmSegment(self.world,
                          arm0_sprite,
                          wposx=win_width/2,
                          wposy=0,
                          angle=0)

        endr, endtheta = arm0.get_end()
        ppos = calculate.PolarPos(endr, endtheta, win_width, win_height)
        wpos = ppos.to_win()
        arm1_sprite = self.spritefactory.from_color(ORANGE, size=(30,200))
        arm1 = ArmSegment(self.world,
                          arm1_sprite,
                          wposx=int(wpos.x),
                          wposy=int(wpos.y),
                          angle=0)
        self.arms = (arm0, arm1)

    def update_device(self, theta0, theta1):
        ppos = self.arms[0].sprite.ppos
        wpos = ppos.to_win()
        angle = np.degrees(-theta0) # pi - theta0
        self.arms[0].update(wpos.x, wpos.y, angle)

        r, theta = self.arms[0].get_end()
        ppos = calculate.PolarPos(r, theta, win_width, win_height)
        wpos = ppos.to_win()
        angle = np.degrees(-theta1)
        self.arms[1].update(wpos.x, wpos.y, angle)


    # run simulation
    def step(self):
        # time frame start
        frame_start = time.monotonic()

        # clear old graphics and update state for next frame
        self.renderer.clear()

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

def check_running():
    events = sdl2.ext.get_events()
    for event in events:
        if event.type == sdl2.SDL_QUIT:
            sdl2.ext.quit()
            return False
    return True

def main():
    vis = SDLWrapper()
    vis.generate_device()
    running = True
    while running:
        t = time.monotonic()
        theta0 = np.pi/4*np.sin(t)
        theta1 = np.pi/4*np.cos(2*t)
        vis.update_device(theta0, theta1)

        text = [[str(vis.arms[0].get_end()[0])[:5],
                 str(vis.arms[0].get_end()[1])[:5]],
                [str(vis.arms[1].sprite.ppos.to_win().x)[:5],
                 str(vis.arms[1].sprite.ppos.to_win().y)[:5]]]
        vis.text(text)

        vis.step()
        # check for window being closed or keypresses and keyboard control
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break

if __name__ == "__main__":
    sys.exit(main())
