# Constant_Throttle_Constant_Speed.py
# METHODS

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------

def initialize_conditions_AOA(segment, state):
    # unpack
    angle_of_attack = segment.angle_of_attack
    air_speed = segment.air_speed
    alt0 = segment.altitude_start
    distance = segment.distance
    t_nondim = state.numerics.dimensionless.control_points
    conditions = state.conditions

    # check for initial altitude
    if alt0 is None:
        if not state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * state.initials.conditions.frames.inertial.position_vector[-1, 2]

    # check for initial altitude
    if distance is None:
        raise AttributeError('Cruise range not set')

    # pack conditions
    conditions.aerodynamics.angle_of_attack[:, 0] = angle_of_attack
    conditions.frames.inertial.velocity_vector[:, 0] = air_speed

# ----------------------------------------------------------------------
#  Update Velocity Vector from Angle of Attack --> Obtain a horizontal speed and climb rate
# ----------------------------------------------------------------------

def update_velocity_vector_from_AOA(segment, state):
    # unpack
    conditions = state.conditions
    v_mag = segment.air_speed
    alpha = segment.angle_of_attack
    theta = state.unknowns.body_angle[:, 0][:, None]

    # Flight path angle
    gamma = theta - alpha #Ensure > 0 for climb!

    # process
    v_x = v_mag * np.cos(gamma)
    v_z = -v_mag * np.sin(gamma)  # z points down
    print v_x,v_z



    # pack
    conditions.frames.inertial.velocity_vector[:, 0] = v_x[:, 0]
    conditions.frames.inertial.velocity_vector[:, 2] = -v_z[:, 0]

    return conditions

def update_differentials_altitude(segment, state):
    """ Segment.update_differentials_altitude(conditions, numerics, unknowns)
        updates the differential operators t, D and I
        must return in dimensional time, with t[0] = 0

        Works with a segment discretized in vertical and horizontal position, and required cruise distance

        Inputs -
            unknowns      - data dictionary of segment free unknowns (body angle and throttle)
            conditions    - data dictionary of segment conditions (set angle of attack and airspeed)
            numerics - data dictionary of non-dimensional differential operators

        Outputs -
            numerics - udpated data dictionary with dimensional numerics

        Assumptions -
            outputed operators are in dimensional time for the current solver iteration
            works with a segment discretized in vertical and horizontal position, distance

    """

    # unpack
    t = state.numerics.dimensionless.control_points
    D = state.numerics.dimensionless.differentiate
    I = state.numerics.dimensionless.integrate

    # Unpack segment initials
    alt0 = segment.altitude_start
    distance = segment.distance
    conditions = state.conditions
    v_mag = segment.air_speed

    r = state.conditions.frames.inertial.position_vector
    v = state.conditions.frames.inertial.velocity_vector

    # check for initial altitude
    if alt0 is None:
        if not state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * state.initials.conditions.frames.inertial.position_vector[-1, 2]

        # get overall time step

    vx = v[:, 0, None]   # Calculated horizontal speed form set cruise speed and angle of attack, and unknown body angle
    vz = -v[:, 2, None]  # Inertial velocity is z down

    #Determine overall timestep
    dx = distance
    dt = dx / np.dot(I[-1, :], vx)[-1] #Seems to me like single small timestep
    # dz = altf - alt0
    # dt = dz / np.dot(I[-1, :], vz)[-1]  # maintain column array

    # Integrate vx to get x-positions
    x0 = r[0,:]
    x = x0 + np.dot(I * dt, vx)

    # Integrate vz to get altitudes over the cruise
    alt = alt0 + np.dot(I * dt, vz)

    # rescale operators
    t = t * dt

    #DO I NEED TO IMPLEMENT THE CLIMB ALTITUDE???

    # pack
    # state.conditions.freestream.altitude[:, 0] = alt
    # state.conditions.frames.inertial.position_vector[:, 2] = -alt  # z points down
    # state.conditions.frames.inertial.velocity_vector[:, 0] = air_speed[:, 0]
    # state.conditions.frames.inertial.time[:, 0] = time[:, 0]

    # pack
    t_initial = state.conditions.frames.inertial.time[0, 0]
    state.conditions.frames.inertial.time[:, 0] = t_initial + t[:, 0]

    conditions.frames.inertial.position_vector[:, 0] = x[:, 0]  # cruise range points - what happens to altitude?
    conditions.frames.inertial.position_vector[:, 2] = -alt[:, 0] # z points down
    conditions.freestream.altitude[:, 0] = alt[:, 0]  # positive altitude in this context

    return



