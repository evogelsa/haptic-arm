import graphics
import controller
import calculate

# define arm first
arm = controller.init()

# user select a vector field
arm.vf = calculate.vector_field_t()
arm.vf.usr_input()
arm.vf.define(arm)

# after vf has been selected
sim = graphics.init(arm)

while True:
    graphics.step(arm, sim)
    controller.step(arm)
