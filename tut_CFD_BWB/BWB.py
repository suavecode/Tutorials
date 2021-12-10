# BWB.py
# 
# Created:  Jan 2017, E. Botero
# Modified: Mar 2018, T. MacDonald

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
assert SUAVE.__version__=='2.5.0', 'These tutorials only work with the SUAVE 2.5.0 release'

import numpy as np
import pylab as plt

from SUAVE.Core import Data, Units

from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Input_Output.OpenVSP import get_vsp_measurements

from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Propulsion import compute_turbofan_geometry
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties

from SUAVE.Plots.Performance.Mission_Plots import *

from copy import deepcopy


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights   

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
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
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

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
    weights = SUAVE.Analyses.Weights.Weights_BWB()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.SU2_Euler()
    aerodynamics.geometry = vehicle
    
    #aerodynamics.process.compute.lift.inviscid.settings.parallel          = True
    #aerodynamics.process.compute.lift.inviscid.settings.processors        = 12  
    aerodynamics.process.compute.lift.inviscid.training_file              = 'base_data_1500.txt'
    aerodynamics.process.compute.lift.inviscid.settings.maximum_iterations = 10
    
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    aerodynamics.settings.half_mesh_flag             = False
    aerodynamics.settings.span_efficiency            = 0.85
    
    aerodynamics.process.compute.lift.inviscid.training.Mach               = np.array([.3, .5, .7, .85]) 
    aerodynamics.process.compute.lift.inviscid.training.angle_of_attack    = np.array([0.,3.,6.]) * Units.deg    
    
    wing_segments = vehicle.wings.main_wing.Segments
    wing_segments.section_1.vsp_mesh = Data()
    wing_segments.section_1.vsp_mesh.inner_radius  = 4.
    wing_segments.section_1.vsp_mesh.outer_radius  = 4.
    wing_segments.section_1.vsp_mesh.inner_length  = .14
    wing_segments.section_1.vsp_mesh.outer_length  = .14
    
    wing_segments.section_2.vsp_mesh = Data()
    wing_segments.section_2.vsp_mesh.inner_radius  = 4.
    wing_segments.section_2.vsp_mesh.outer_radius  = 4.
    wing_segments.section_2.vsp_mesh.inner_length  = .14
    wing_segments.section_2.vsp_mesh.outer_length  = .14
    
    wing_segments.section_3.vsp_mesh = Data()
    wing_segments.section_3.vsp_mesh.inner_radius  = 4.
    wing_segments.section_3.vsp_mesh.outer_radius  = 4.
    wing_segments.section_3.vsp_mesh.inner_length  = .14
    wing_segments.section_3.vsp_mesh.outer_length  = .14
    
    wing_segments.section_4.vsp_mesh = Data()
    wing_segments.section_4.vsp_mesh.inner_radius  = 4.
    wing_segments.section_4.vsp_mesh.outer_radius  = 2.8
    wing_segments.section_4.vsp_mesh.inner_length  = .14
    wing_segments.section_4.vsp_mesh.outer_length  = .14      
    
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
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
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Boeing_BWB_450'    

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.max_takeoff               = 823000. * Units.lb
    vehicle.mass_properties.takeoff                   = 823000. * Units.lb
    vehicle.mass_properties.max_zero_fuel             = 0.9 * vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.cargo                     = 00.  * Units.kilogram   

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5
    vehicle.envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 7840. * 2 * Units.feet**2       
    vehicle.passengers             = 450.
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "medium range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 289.**2 / (7840. * 2)
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.0138
    wing.spans.projected         = 289.0 * Units.feet  
    wing.chords.root             = 145.0 * Units.feet
    wing.chords.tip              = 3.5  * Units.feet
    wing.chords.mean_aerodynamic = 80. * Units.feet
    wing.areas.reference         = 7840. * 2 * Units.feet**2
    wing.sweeps.quarter_chord    = 33. * Units.degrees
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.dihedral                = 2.5 * Units.degrees
    wing.origin                  = [[0.,0.,0]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0

    segment = SUAVE.Components.Wings.Segment()

    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 1.
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 40.0 * Units.degrees
    segment.thickness_to_chord    = 0.165
    segment.vsp_mesh              = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14    
    wing.Segments.append(segment)    
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                      = 'section_2'
    segment.percent_span_location    = 0.052
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.921
    segment.dihedral_outboard        = 0.   * Units.degrees
    segment.sweeps.quarter_chord     = 52.5 * Units.degrees
    segment.thickness_to_chord       = 0.167
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14     
    wing.Segments.append(segment)   

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                      = 'section_3'
    segment.percent_span_location    = 0.138
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.76
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 36.9 * Units.degrees  
    segment.thickness_to_chord       = 0.171
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14     
    wing.Segments.append(segment)   
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                      = 'section_4'
    segment.percent_span_location    = 0.221
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.624
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 30.4 * Units.degrees    
    segment.thickness_to_chord       = 0.175
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 2.8
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14     
    wing.Segments.append(segment)       
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_5'
    segment.percent_span_location = 0.457
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.313
    segment.dihedral_outboard     = 1.85  * Units.degrees
    segment.sweeps.quarter_chord  = 30.85 * Units.degrees
    segment.thickness_to_chord    = 0.118
    wing.Segments.append(segment)       
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_6'
    segment.percent_span_location = 0.568
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.197
    segment.dihedral_outboard     = 1.85 * Units.degrees
    segment.sweeps.quarter_chord  = 34.3 * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.Segments.append(segment)     
    
    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'section_7'
    segment.percent_span_location = 0.97
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.086
    segment.dihedral_outboard     = 73. * Units.degrees
    segment.sweeps.quarter_chord  = 55. * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.Segments.append(segment)      

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.0241
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 0. * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.Segments.append(segment)  
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)         

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Nacelle  
    # ------------------------------------------------------------------
    nacelle                       = SUAVE.Components.Nacelles.Nacelle()
    nacelle.diameter              = 3.96 * Units.meters 
    nacelle.length                = 289. * Units.inches
    nacelle.tag                   = 'nacelle' 
    nacelle.origin                = [[123.0 *Units.feet, 25.0*Units.feet, 6.5*Units.feet]]
    nacelle.Airfoil.naca_4_series_airfoil = '0012'
        

    # ------------------------------------------------------------------
    #   Turbofan Network
    # ------------------------------------------------------------------
    #instantiate the gas turbine network
    turbofan     = SUAVE.Components.Energy.Networks.Turbofan()
    turbofan.tag = 'turbofan1'

    # setup
    turbofan.number_of_engines = 3.0
    turbofan.bypass_ratio      = 8.1
    turbofan.origin            = [[133.0 *Units.feet, 25.0*Units.feet, 6.5*Units.feet],[145.0 *Units.feet, 0.0*Units.feet, 6.5*Units.feet],[133.0 *Units.feet, -25.0*Units.feet, 6.5*Units.feet]]
    
    # working fluid
    turbofan.working_fluid = SUAVE.Attributes.Gases.Air()
    
    # ------------------------------------------------------------------
    #   Component 1 - Ram
    
    # to convert freestream static to stagnation quantities
    # instantiate
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'
    # add to the network
    turbofan.append(ram)
    
    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle
    
    # instantiate
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'
    # setup
    inlet_nozzle.polytropic_efficiency = 1.0
    inlet_nozzle.pressure_ratio        = 1.0
    # add to network
    turbofan.append(inlet_nozzle)
    
    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor
    
    # instantiate
    compressor = SUAVE.Components.Energy.Converters.Compressor()
    compressor.tag = 'low_pressure_compressor'
    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 1.1
    # add to network
    turbofan.append(compressor)
    
    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    # instantiate
    compressor = SUAVE.Components.Energy.Converters.Compressor()
    compressor.tag = 'high_pressure_compressor'
    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 23.0
    #compressor.hub_to_tip_ratio      = 0.325
    # add to network
    turbofan.append(compressor)
    
    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()
    turbine.tag='low_pressure_turbine'
    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93
    # add to network
    turbofan.append(turbine)
    
    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine
    
    # instantiate
    turbine = SUAVE.Components.Energy.Converters.Turbine()
    turbine.tag='high_pressure_turbine'
    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93
    # add to network
    turbofan.append(turbine)
    
    # ------------------------------------------------------------------
    #  Component 7 - Combustor
    
    # instantiate
    combustor = SUAVE.Components.Energy.Converters.Combustor()
    combustor.tag = 'combustor'
    # setup
    combustor.efficiency                = 1.0
    combustor.alphac                    = 1.0
    combustor.turbine_inlet_temperature = 1592. * Units.kelvin
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()
    # add to network
    turbofan.append(combustor)
    
    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle
    
    # instantiate
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()
    nozzle.tag = 'core_nozzle'
    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99 
    # add to network
    turbofan.append(nozzle)
    
    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle
    
    # instantiate
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()
    nozzle.tag = 'fan_nozzle'
    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99 
    # add to network
    turbofan.append(nozzle)
    
    # ------------------------------------------------------------------
    #  Component 10 - Fan
    
    # instantiate
    fan = SUAVE.Components.Energy.Converters.Fan()
    fan.tag = 'fan'
    # setup
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.58
    # add to network
    turbofan.append(fan)
    
    # ------------------------------------------------------------------
    #Component 10 : thrust (to compute the thrust)
    thrust = SUAVE.Components.Energy.Processes.Thrust()
    thrust.tag ='compute_thrust'
    
    #total design thrust (includes all the engines)
    thrust.total_design  = 2.0*512000 * Units.N
    thrust.bypass_ratio  = 8.4
    
    #design sizing conditions
    altitude      = 0. * Units.km
    mach_number   = 0.01
    
    # add to network
    turbofan.thrust = thrust
    
    #size the turbofan
    turbofan_sizing(turbofan,mach_number,altitude)
    
    #computing the engine length and diameter
    compute_turbofan_geometry(turbofan,nacelle)
    
    vehicle.append_component(turbofan)  
    
    # Finish adding all the nacelles
    
    nacelle_2                     = deepcopy(nacelle)
    nacelle_2.tag                 = 'nacelle_2' 
    nacelle_2.origin              = [[135.0 *Units.feet, 0.0*Units.feet, 6.5*Units.feet]]     
     
    nacelle_3                     = deepcopy(nacelle)
    nacelle_3.tag                 = 'nacelle_3'
    nacelle_3.origin              = [[123.0 *Units.feet, -25.0*Units.feet, 6.5*Units.feet]]   
    
    vehicle.append_component(nacelle) 
    vehicle.append_component(nacelle_2) 
    vehicle.append_component(nacelle_3)     


    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

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
    
    write(vehicle,base_config.tag) 

    return configs

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)

    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)

    # Drag Components
    plot_drag_components(results, line_style)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)

    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)           
        
    return

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # Areas
    wetted_areas = get_vsp_measurements(base.tag)

    for wing in base.wings:
        wing.areas.wetted   = wetted_areas[wing.tag]
        wing.areas.exposed  = wetted_areas[wing.tag]
        wing.areas.affected = 0.6 * wing.areas.wetted 

    # diff the new data
    base.store_diff()

    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.airport    = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------
    #   First Climb Segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.base )
    
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 3. * Units.deg      

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 125.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']

    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.base )

    segment.altitude_end   = 8.0   * Units.km
    segment.air_speed      = 190.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Climb Segment
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 35000 * Units.feet
    segment.air_speed    = 226.0  * Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------    
    #   Cruise Segment
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.base )

    segment.mach     = 0.78
    segment.distance = 6500 * Units.nautical_mile

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 8.0   * Units.km
    segment.air_speed    = 220.0 * Units['m/s']
    segment.descent_rate = 4.5   * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 6.0   * Units.km
    segment.air_speed    = 195.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 4.0   * Units.km
    segment.air_speed    = 170.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 2.0   * Units.km
    segment.air_speed    = 150.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']


    # add to mission
    mission.append_segment(segment)



    # ------------------------------------------------------------------
    #   Fifth Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5"

    segment.analyses.extend( analyses.base )

    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 145.0 * Units['m/s']
    segment.descent_rate = 3.0   * Units['m/s']

    # append to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission 

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions  

if __name__ == '__main__': 
    main()    
    plt.show()

