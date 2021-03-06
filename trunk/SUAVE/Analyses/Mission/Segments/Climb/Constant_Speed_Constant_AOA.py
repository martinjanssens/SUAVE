# Constant_Speed_Constant_AOA.py
#ANALYSIS

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports
from SUAVE.Analyses.Mission.Segments import Aerodynamic
from SUAVE.Analyses.Mission.Segments import Conditions

from SUAVE.Methods.Missions import Segments as Methods
from Unknown_Throttle import Unknown_Throttle


from SUAVE.Analyses import Process

# Units
from SUAVE.Core import Units


# ----------------------------------------------------------------------
#  Segment
# ----------------------------------------------------------------------

class Constant_Speed_Constant_AOA(Aerodynamic):
    def __defaults__(self):
        # --------------------------------------------------------------
        #   User inputs
        # --------------------------------------------------------------
        self.altitude_start = 10. * Units.km
        self.distance = 10. * Units.km
        self.angle_of_attack = 0.5 * Units.deg
        self.air_speed = 100 * Units.m / Units.s
        self.altitude_end = None

        # --------------------------------------------------------------
        #   State
        # --------------------------------------------------------------

        # conditions
        self.state.conditions.update(Conditions.Aerodynamics())

        # initials and unknowns
        ones_row = self.state.ones_row
        self.state.unknowns.body_angle = ones_row(1) * 4.0 * Units.deg
        self.state.unknowns.throttle = ones_row(1) * 0.9 #??????
        self.state.residuals.forces = ones_row(2) * 0.0

        # --------------------------------------------------------------
        #   The Solving Process
        # --------------------------------------------------------------

        # --------------------------------------------------------------
        #   Initialize - before iteration
        # --------------------------------------------------------------
        initialize = self.process.initialize
        initialize.clear()

        initialize.expand_state = Methods.expand_state
        initialize.differentials = Methods.Common.Numerics.initialize_differentials_dimensionless
        initialize.conditions = Methods.Climb.Constant_Speed_Constant_AOA.initialize_conditions_AOA
        initialize.velocities = Methods.Climb.Constant_Speed_Constant_AOA.update_velocity_vector_from_AOA
        initialize.differentials_altitude = Methods.Climb.Constant_Speed_Constant_AOA.update_differentials_altitude

        # --------------------------------------------------------------
        #   Converge - starts iteration
        # --------------------------------------------------------------
        converge = self.process.converge
        converge.clear()

        converge.converge_root = Methods.converge_root

        # --------------------------------------------------------------
        #   Iterate - this is iterated
        # --------------------------------------------------------------
        iterate = self.process.iterate
        iterate.clear()

        #Update Initials
        iterate.initials = Process()
        iterate.initials.time = Methods.Common.Frames.initialize_time
        iterate.initials.weights = Methods.Common.Weights.initialize_weights
        iterate.initials.inertial_position = Methods.Common.Frames.initialize_inertial_position
        iterate.initials.planet_position = Methods.Common.Frames.initialize_planet_position

        #Unpack Unknowns
        iterate.unpack_unknowns = Methods.Climb.Common.unpack_unknowns

        #Update Conditions
        iterate.conditions = Process()
        iterate.conditions.velocities = Methods.Climb.Constant_Speed_Constant_AOA.update_velocity_vector_from_AOA
        iterate.conditions.differentials_a = Methods.Climb.Constant_Speed_Constant_AOA.update_differentials_altitude
        iterate.conditions.differentials_b = Methods.Common.Numerics.update_differentials_time
        iterate.conditions.acceleration = Methods.Common.Frames.update_acceleration
        iterate.conditions.altitude = Methods.Common.Aerodynamics.update_altitude
        iterate.conditions.atmosphere = Methods.Common.Aerodynamics.update_atmosphere
        iterate.conditions.gravity = Methods.Common.Weights.update_gravity
        iterate.conditions.freestream = Methods.Common.Aerodynamics.update_freestream
        iterate.conditions.orientations = Methods.Common.Frames.update_orientations
        iterate.conditions.aerodynamics = Methods.Common.Aerodynamics.update_aerodynamics
        iterate.conditions.stability = Methods.Common.Aerodynamics.update_stability
        iterate.conditions.propulsion = Methods.Common.Energy.update_thrust
        iterate.conditions.weights = Methods.Common.Weights.update_weights
        iterate.conditions.forces = Methods.Common.Frames.update_forces
        iterate.conditions.planet_position = Methods.Common.Frames.update_planet_position

        #Solve Residuals
        iterate.residuals = Process()
        iterate.residuals.total_forces = Methods.Climb.Common.residual_total_forces
        #self.state.conditions.propulsion.throttle = 0.78 * ones_row(1)
        # --------------------------------------------------------------
        #   Finalize - after iteration
        # --------------------------------------------------------------
        finalize = self.process.finalize
        finalize.clear()

        # Post Processing
        finalize.post_process = Process()
        finalize.post_process.inertial_position = Methods.Common.Frames.integrate_inertial_horizontal_position
        finalize.post_process.stability = Methods.Common.Aerodynamics.update_stability

        return

