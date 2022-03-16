# QS_tutorial.py
# 
# Created:  Feb 2022, E. Botero
# Modified: 

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units

import pylab as plt

from copy import deepcopy
import os

from SUAVE.Plots.Performance.Mission_Plots import *  
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_polars import compute_airfoil_polars

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()
    
    configs.finalize()
    analyses.finalize()    
    
    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plot results    
    plot_mission(results)
    
    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    
    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    
    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    
    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    
    return configs, analyses

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'tail_sitter'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff         = 0.82 * Units.kg
    vehicle.mass_properties.operating_empty = 0.82 * Units.kg
    vehicle.mass_properties.max_takeoff     = 0.82 * Units.kg
    
    # basic parameters
    vehicle.reference_area                  = 0.1668 
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 1.03 * Units.m
    wing.aspect_ratio            = (wing.spans.projected**2)/wing.areas.reference 
    wing.sweeps.quarter_chord    = 5.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.dynamic_pressure_ratio  = 1.0
    wing.chords.mean_aerodynamic = 0.162 * Units.m
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.high_lift               = False
    wing.vertical                = False
    wing.symmetric               = True

    # add to vehicle
    vehicle.append_component(wing)    

    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------
    
    # build network
    net = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.number_of_propeller_engines = 4.
    net.voltage                     = 12.3
    net.identical_propellers        = True
    
    # Component 1 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc
    
    # Component 2 the Propeller
    
    # Design the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades    = 2.0
    prop.freestream_velocity = 15.0 # freestream m/s
    prop.angular_velocity    = 7500. * Units['rpm']
    prop.tip_radius          = 4.    * Units.inch
    prop.hub_radius          = 0.125 * Units.inch
    prop.design_Cl           = 0.7
    prop.design_altitude     = 0.1 * Units.km
    prop.design_power        = 200. * Units.watts
    prop                     = propeller_design(prop)
    
    origins = [[0., 0.15, -0.05], [0., -0.15, -0.05], [0., .35, 0.05], [0., 0.35, 0.05]]
    
    
    for ii in range(4):
        rotor          = deepcopy(prop)
        rotor.tag      = 'propeller'
        rotor.origin   = [origins[ii]]
        net.propellers.append(rotor)    

    # Component 3 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor()
    motor.speed_constant       = 1500. * Units['rpm'] # RPM/volt converted to (rad/s)/volt 
    motor = size_from_kv(motor)
    motor.gear_ratio           = 1.  # Gear ratio
    motor.gearbox_efficiency   = 1.  # Gear box efficiency
    motor.expected_current     = 10. # Expected current
    motor.propeller_radius     = prop.tip_radius

    for ii in range(4):
        rotor_motor = deepcopy(motor)
        rotor_motor.tag    = 'motor'
        rotor_motor.origin = [origins[ii]]
        net.propeller_motors.append(rotor_motor)

    
    # Component 4 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 0. #Watts 
    payload.mass_properties.mass = 0.0 * Units.kg
    net.payload                  = payload
    
    # Component 5 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 2. #Watts  
    net.avionics        = avionics      

    # Component 6 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 0.17 * Units.kg
    bat.specific_energy      = 175.*Units.Wh/Units.kg
    bat.resistance           = 0.003
    bat.max_voltage          = 11.1
    initialize_from_mass(bat)
    net.battery              = bat
    
    # add the solar network to the vehicle
    vehicle.append_component(net)  

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    
    configs = SUAVE.Components.Configs.Config.Container()
    
    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    return configs


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    
    analyses = SUAVE.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis
    
    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()
    
    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights()
    weights.settings.empty_weight_method = \
        SUAVE.Methods.Weights.Correlations.UAV.empty
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.AERODAS()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    aerodynamics.settings.maximum_lift_coefficient   = 1.5
    analyses.append(aerodynamics)    
    
    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
    
    # done!
    return analyses    

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses,vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'The Test Mission'

    mission.atmosphere  = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet      = SUAVE.Attributes.Planets.Earth()
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    
    # base segment
    base_segment = Segments.Segment()   
    base_segment.process.initialize.initialize_battery       = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.process.iterate.conditions.planet_position  = SUAVE.Methods.skip
    base_segment.state.numerics.number_control_points        = 3

    #------------------------------------------------------------------    
    #  Climb Hover
    #------------------------------------------------------------------    
    
    segment = SUAVE.Analyses.Mission.Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes   
    ones_row = segment.state.ones_row
    segment.battery_energy  = vehicle.networks.battery_propeller.battery.max_energy
    segment.altitude_start  = 0.
    segment.altitude_end    = 100. * Units.m
    segment.climb_rate      = 3.  * Units.m / Units.s 
    segment.air_speed       = 3.  * Units.m / Units.s
    segment.state.unknowns.body_angle  = ones_row(1) * 90. *Units.deg
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.1)    
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------    
    #   Hover
    # ------------------------------------------------------------------    
    
    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Hover_1"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes     
    segment.time = 60* Units.seconds
    segment.state.conditions.frames.body.inertial_rotations[:,1] = ones_row(1) * 90.*Units.deg 
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                         initial_power_coefficient = 0.07)    
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------    
    #   Hover Transition
    # ------------------------------------------------------------------     
    
    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Transition_to_Cruise"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes       
    segment.acceleration      = 1.5 * Units['m/s/s']
    segment.air_speed_initial = 0.0
    segment.air_speed_final   = 15.0 
    segment.altitude          = 100. * Units.m
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.07)
    
    mission.append_segment(segment)      
           

    # ------------------------------------------------------------------    
    #   Cruise
    # ------------------------------------------------------------------     
    
    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes     
    segment.distance  = 3.  * Units.km
    segment.air_speed = 15. * Units.m/Units.s
    segment.altitude  = 100. * Units.m
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.04)

    mission.append_segment(segment)            
    
    # ------------------------------------------------------------------    
    #   Hover Transition
    # ------------------------------------------------------------------     
    
    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "Transition_to_hover"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes       
    segment.acceleration      = -0.5 * Units['m/s/s']
    segment.air_speed_initial = 15.0
    segment.air_speed_final   = 0.0 
    segment.altitude = 100. * Units.m
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.07)
    
    mission.append_segment(segment)  
    
    # ------------------------------------------------------------------    
    #   Hover
    # ------------------------------------------------------------------    
    
    segment = SUAVE.Analyses.Mission.Segments.Hover.Hover(base_segment)
    segment.tag = "Hover_2"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes     
    segment.time = 60* Units.seconds
    segment.state.conditions.frames.body.inertial_rotations[:,1] = ones_row(1)* 90.*Units.deg
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.07)
    
    mission.append_segment(segment)        
    
    # ------------------------------------------------------------------    
    #   Descent Hover
    # ------------------------------------------------------------------    
    
    segment = SUAVE.Analyses.Mission.Segments.Hover.Descent(base_segment)
    segment.tag = "Descent"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.base)
    
    # segment attributes     
    segment.altitude_end = 0. * Units.m
    segment.descent_rate = 3. * Units.m / Units.s   
    segment.state.conditions.frames.body.inertial_rotations[:,1] = ones_row(1)* 90.*Units.deg
    
    segment = vehicle.networks.battery_propeller.add_unknowns_and_residuals_to_segment(segment,\
                                                                                       initial_power_coefficient = 0.06)
    
    mission.append_segment(segment)       
    
    #------------------------------------------------------------------    
    #   Mission definition complete    
    #-------------------------------------------------------------------
    
    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()
    
    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------
    
    missions.base = base_mission
    
    # done!
    return missions  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------
def plot_mission(results):


    # Plot Flight Conditions 
    plot_flight_conditions(results) 
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)  
    
    # Plot Aircraft Flight Speed
    plot_aircraft_velocities(results)
    
    # Plot Aircraft Electronics
    plot_battery_pack_conditions(results)
    
    # Plot Propeller Conditions 
    plot_propeller_conditions(results) 
    
    # Plot Electric Motor and Propeller Efficiencies 
    plot_eMotor_Prop_efficiencies(results)

    # Plot propeller Disc and Power Loading
    plot_disc_power_loading(results)  
    
    return     

if __name__ == '__main__':
    main()
    
    plt.show()