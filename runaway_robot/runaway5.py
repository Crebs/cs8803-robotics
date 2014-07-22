# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

import numpy as np

from runaway_robot.robot import *
from matrix import *


def able_to_chase(hunter_p, robot_p, max_distance):
    return distance_between(hunter_p, robot_p) < max_distance

def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # identity matrix
    I = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.],
                [0., 0., 1., 0., 0.],
                [0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 1.]])

    #motion update matrix
    H = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.]])

    #measurement noise
    R = matrix([[measurement_noise, 0.],
                [0., measurement_noise]])

    z = matrix([[measurement[0]],
                [measurement[1]]])

    u = matrix([[0.],
                [0.],
                [0.],
                [0.],
                [0.]])

    if OTHER is not None and 'X' not in OTHER:
        last_measurement = OTHER['last_measurement']

        angle = atan2(measurement[0] - last_measurement[0], measurement[1] - last_measurement[1])
        if 'last_angle' not in OTHER:
            OTHER['last_angle'] = angle
            xy_estimate = [1., 1.]
            OTHER['last_measurement'] = measurement
            return xy_estimate, OTHER
        else:
            turning_angle = angle - OTHER['last_angle']

    elif OTHER is None:
        OTHER = {'last_measurement': measurement}
        return [1.,1.], OTHER


    if 'X' in OTHER:
        X = OTHER['X']
        P = OTHER['P']
        x,y, radius, OTHER = regression(measurement, OTHER)
        weight = distance_between([x,y], measurement)
        weight= abs(radius - weight)/radius

        R = matrix([[measurement_noise, 0.],
            [0., measurement_noise ]])

        print 'weight is %s' %weight
        print 'radius is %s' %radius
    else:
        weight = 1

        X = matrix([[measurement[0]],
                    [measurement[1]],
                    [1.],
                    [turning_angle],
                    [1.5]])
        #convariance matrix
        P = matrix([[1000, 0., 0., 0., 0.],
                    [0., 1000, 0., 0., 0.],
                    [0., 0., 1000, 0., 0.],
                    [0., 0., 0., 1000, 0.],
                    [0., 0., 0., 0., 1000]
        ])

        R = matrix([[measurement_noise, 0.],
                [0., measurement_noise ]])

    #measurement update
    Y = z - (H * X)
    S = H * P * H.transpose() + R
    S_np = np.array(S.value)
    inv_S = np.linalg.pinv(S_np)
    K = P * H.transpose() * matrix(inv_S)

    #print 'weight is %s' %weight
    #K.value = [[v/weight for v in row] for row in K.value]


    X = X + (K * Y)
    P = (I - (K * H)) * P

    #Prediction
    x = X.value[0][0]
    y = X.value[1][0]
    angle = X.value[2][0]
    turning_angle = X.value[3][0]
    distance = X.value[4][0]



    new_X = [
        [x + distance * sin(angle+turning_angle)],
        [y + distance * cos(angle+turning_angle)],
        [angle_trunc(angle+turning_angle)],
        [turning_angle],
        [distance],
    ]

    update_row0 = [
        1.,
        0.,
        distance * cos(angle+turning_angle),
        distance * cos(angle+turning_angle),
        sin(angle+turning_angle),
    ]

    update_row1 = [
        0.,
        1.,
        -distance * sin(turning_angle+angle),
        -distance * sin(turning_angle+angle),
        cos(angle+turning_angle)
    ]

    updated_X = [
        update_row0,
        update_row1,
        [0., 0., 1., 1., 0.],
        [0., 0., 0., 1., 0.],
        [0., 0., 0., 0., 1.],
    ]

    A = matrix(updated_X)

    P = A * P * A.transpose()

    X = matrix(new_X)

    xy_estimate = [X.value[0][0], X.value[1][0]]
    #OTHER = {'X': X, 'P': P}
    OTHER['X'] = X
    OTHER['P'] = P

    #P.show()

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #X.show()

    #print '----------------------------------------------'
    return xy_estimate, OTHER

def find_ambush_point(hunter_position, robotpos, max_distance, OTHER):
    print 'Planning ambush..'

    OTHER['chase_counter'] = 0
    target_steps = 1
    next_xy = robotpos

    X = OTHER['X']
    P = OTHER['P']
    X_copy = matrix([[v for v in c] for c in X.value])
    P_copy = matrix([[v for v in c] for c in P.value])

    OTHER2 = {'X': X_copy, 'P': P_copy}

    while True:
        if distance_between(hunter_position, next_xy)/max_distance < target_steps:
            next_xy2, OTHER2 = estimate_next_pos(next_xy, OTHER2)
            next_xy = [(next_xy[0] + next_xy2[0])/2, (next_xy[1] + next_xy2[1])/2]
            break
        else:
            next_xy, OTHER2 = estimate_next_pos(next_xy, OTHER2)
            target_steps += 1

    return next_xy

def regression(target_measurement, OTHER):
    global estimation
    if 'LH' not in OTHER:
        OTHER['LH'] =  np.array([[0. for _ in range(3)] for x in range(3)])
        OTHER['RH'] =  np.array([[0.] for _ in range(3)])
        estimation = [1,1]
        OTHER['counter'] = 0
        OTHER['points'] = []

        estimation  = [1,1]
        return 1,1,1,OTHER

    else:
        OTHER['points'].append(target_measurement)
        OTHER['counter'] += 1
        LH = OTHER['LH']
        RH = OTHER['RH']
        LH[0][0] += target_measurement[0] ** 2
        LH[0][1] += target_measurement[0] * target_measurement[1]
        LH[0][2] += target_measurement[0]
        LH[1][0] += target_measurement[0] * target_measurement[1]
        LH[1][1] += target_measurement[1] **2
        LH[1][2] += target_measurement[1]
        LH[2][0] += target_measurement[0]
        LH[2][1] += target_measurement[1]
        LH[2][2] += 1

        RH[0][0] += target_measurement[0] * (target_measurement[0]**2 + target_measurement[1]**2)
        RH[1][0] += target_measurement[1] * (target_measurement[0]**2 + target_measurement[1]**2)
        RH[2][0] += target_measurement[0]**2 + target_measurement[1]**2

        if OTHER['counter'] < 10:
            return 1,1,1,OTHER


        parameters = np.linalg.solve(LH, RH)
        p1 = parameters[0][0]
        p2 = parameters[1][0]
        p3 = parameters[2][0]

        x = p1/2.
        y = p2/2.
        radius = sqrt((4 * p3 + p1**2 + p2**2))/2
        estimation = [x,y]
        print 'centre is'
        print estimation
        return x,y,radius, OTHER

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    global estimation
    estimation = [1,1]
    next_xy, OTHER = estimate_next_pos(target_measurement, OTHER)
    if 'X' in OTHER:
        OTHER['X'].show()
    destination = next_xy

    angle = atan2(destination[1]-hunter_position[1], destination[0]-hunter_position[0])
    desired_heading = angle_trunc(angle - hunter_heading)
    distance_to_move = distance_between(destination, hunter_position)
    return desired_heading, distance_to_move, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def demo_grading2(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    global estimation

    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')

    estimation_robot =  turtle.Turtle()
    estimation_robot.shape('circle')
    estimation_robot.color('black')
    estimation_robot.resizemode('user')
    estimation_robot.shapesize(0.1, 0.1, 0.1)
    estimation_robot.penup()


    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        if estimation:
            estimation_robot.goto(estimation[0]*size_multiplier, estimation[1]*size_multiplier-100)

        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
# measurement_noise = 2.0*target.distance # VERY NOISY!!
measurement_noise = .5*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading2(hunter, target, next_move)





