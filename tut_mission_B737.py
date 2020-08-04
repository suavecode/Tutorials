# tut_mission_B737.py
# 
# Created:  Aug 2014, SUAVE Team
# Modified: Aug 2017, SUAVE Team
#           Mar 2020, E. Botero

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# General Python Imports
import numpy as np
# Numpy is a commonly used mathematically computing package. It contains many frequently used
# mathematical functions and is faster than native Python, especially when using vectorized
# quantities.
import matplotlib.pyplot as plt
# Matplotlib's pyplot can be used to generate a large variety of plots. Here it is used to create
# visualizations of the aircraft's performance throughout the mission.

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units 
# The Data import here is a native SUAVE data structure that functions similarly to a dictionary.
#   However, iteration directly returns values, and values can be retrieved either with the 
#   typical dictionary syntax of "entry['key']" or the more class-like "entry.key". For this to work
#   properly, all keys must be strings.
# The Units import is used to allow units to be specified in the vehicle setup (or elsewhere).
#   This is because SUAVE functions generally operate using metric units, so inputs must be 
#   converted. To use a length of 20 feet, set l = 20 * Units.ft . Additionally, to convert to SUAVE
#   output back to a desired units, use l_ft = l_m / Units.ft
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing
# Rather than conventional sizing, this script builds the turbofan energy network. This process is
# covered in more detail in a separate tutorial. It does not size the turbofan geometry.


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    """This function gets the vehicle configuration, analysis settings, and then runs the mission.
    Once the mission is complete, the results are plotted."""
    
    # Extract vehicle configurations and the analysis settings that go with them
    configs, analyses = full_setup()

    # Size each of the configurations according to a given set of geometry relations
    simple_sizing(configs)

    # Perform operations needed to make the configurations and analyses usable in the mission
    configs.finalize()
    analyses.finalize()

    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # Performance a mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # Plot all mission results, including items such as altitude profile and L/D
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    """This function gets the baseline vehicle and creates modifications for different 
    configurations, as well as the mission and analyses to go with those configurations."""

    # Collect baseline vehicle data and changes when using different configuration settings
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # Get the analyses to be used when different configurations are evaluated
    configs_analyses = analyses_setup(configs)

    # Create the mission that will be flown
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    # Add the analyses to the proper containers
    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = SUAVE.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Tube_Wing()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors 
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

    return analyses    

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    """This is the full physical definition of the vehicle, and is designed to be independent of the
    analyses that are selected."""
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Boeing_737-800'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # Vehicle level mass properties
    # The maximum takeoff gross weight is used by a number of methods, most notably the weight
    # method. However, it does not directly inform mission analysis.
    vehicle.mass_properties.max_takeoff               = 79015.8 * Units.kilogram 
    # The takeoff weight is used to determine the weight of the vehicle at the start of the mission
    vehicle.mass_properties.takeoff                   = 79015.8 * Units.kilogram   
    # Operating empty may be used by various weight methods or other methods. Importantly, it does
    # not constrain the mission analysis directly, meaning that the vehicle weight in a mission
    # can drop below this value if more fuel is needed than is available.
    vehicle.mass_properties.operating_empty           = 62746.4 * Units.kilogram 
    # The maximum zero fuel weight is also used by methods such as weights
    vehicle.mass_properties.max_zero_fuel             = 62732.0 * Units.kilogram
    # Cargo weight typically feeds directly into weights output and does not affect the mission
    vehicle.mass_properties.cargo                     = 10000.  * Units.kilogram   
    
    # Envelope properties
    # These values are typical FAR values for a transport of this type
    vehicle.envelope.ultimate_load = 3.75
    vehicle.envelope.limit_load    = 2.5

    # Vehicle level parameters
    # The vehicle reference area typically matches the main wing reference area 
    vehicle.reference_area         = 124.862 * Units['meters**2']  
    # Number of passengers, control settings, and accessories settings are used by the weights
    # methods
    vehicle.passengers             = 170
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "medium range"

    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------ 
    
    # The settings here can be used for noise analysis, but are not used in this tutorial
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    
    landing_gear.main_tire_diameter = 1.12000 * Units.m
    landing_gear.nose_tire_diameter = 0.6858 * Units.m
    landing_gear.main_strut_length  = 1.8 * Units.m
    landing_gear.nose_strut_length  = 1.3 * Units.m
    landing_gear.main_units  = 2    # Number of main landing gear
    landing_gear.nose_units  = 1    # Number of nose landing gear
    landing_gear.main_wheels = 2    # Number of wheels on the main landing gear
    landing_gear.nose_wheels = 2    # Number of wheels on the nose landing gear      
    vehicle.landing_gear = landing_gear

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    
    # This main wing is approximated as a simple trapezoid. A segmented wing can also be created if
    # desired. Segmented wings appear in later tutorials, and a version of the 737 with segmented
    # wings can be found in the SUAVE testing scripts.
    
    # SUAVE allows conflicting geometric values to be set in terms of items such as aspect ratio
    # when compared with span and reference area. Sizing scripts may be used to enforce 
    # consistency if desired.
    
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.aspect_ratio            = 10.18
    # Quarter chord sweep is used as the driving sweep in most of the low fidelity analysis methods.
    # If a different known value (such as leading edge sweep) is given, it should be converted to
    # quarter chord sweep and added here. In some cases leading edge sweep will be used directly as
    # well, and can be entered here too.
    wing.sweeps.quarter_chord    = 25 * Units.deg
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.1
    wing.spans.projected         = 34.32 * Units.meter
    wing.chords.root             = 7.760 * Units.meter
    wing.chords.tip              = 0.782 * Units.meter
    wing.chords.mean_aerodynamic = 4.235 * Units.meter
    wing.areas.reference         = 124.862 * Units['meters**2']  
    wing.twists.root             = 4.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.origin                  = [13.61,0,-1.27] # meters
    wing.vertical                = False
    wing.symmetric               = True
    # The high lift flag controls aspects of maximum lift coefficient calculations
    wing.high_lift               = True
    # The dynamic pressure ratio is used in stability calculations
    wing.dynamic_pressure_ratio  = 1.0
    
    # ------------------------------------------------------------------
    #   Main Wing Control Surfaces
    # ------------------------------------------------------------------
    
    # Information in this section is used for high lift calculations and when conversion to AVL
    # is desired.
    
    # Deflections will typically be specified separately in individual vehicle configurations.
    
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.10 
    flap.span_fraction_end     = 0.75   
    flap.deflection            = 0.0 * Units.degrees
    # Flap configuration types are used in computing maximum CL and noise
    flap.configuration_type    = 'double_slotted'
    flap.chord_fraction        = 0.30   
    wing.append_control_surface(flap)   
        
    slat                       = SUAVE.Components.Wings.Control_Surfaces.Slat() 
    slat.tag                   = 'slat' 
    slat.span_fraction_start   = 0.324 
    slat.span_fraction_end     = 0.963     
    slat.deflection            = 0.0 * Units.degrees
    slat.chord_fraction        = 0.1  	 
    wing.append_control_surface(slat)  
        
    aileron                       = SUAVE.Components.Wings.Control_Surfaces.Aileron() 
    aileron.tag                   = 'aileron' 
    aileron.span_fraction_start   = 0.7 
    aileron.span_fraction_end     = 0.963 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16    
    wing.append_control_surface(aileron)    
    
    # Add to vehicle
    vehicle.append_component(wing)    

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    
    wing.aspect_ratio            = 6.16     
    wing.sweeps.quarter_chord    = 40 * Units.deg
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.2
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 14.2 * Units.meter
    wing.chords.root             = 4.7  * Units.meter
    wing.chords.tip              = .955 * Units.meter
    wing.chords.mean_aerodynamic = 8.0  * Units.meter
    wing.areas.reference         = 32.488   * Units['meters**2']  
    wing.twists.root             = 3.0 * Units.degrees
    wing.twists.tip              = 3.0 * Units.degrees  
    wing.origin                  = [32.83,0,1.14] # meters
    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9  
    
    # Add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio            = 1.91
    wing.sweeps.quarter_chord    = 25. * Units.deg
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.25
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 7.777 * Units.meter
    wing.chords.root             = 8.19  * Units.meter
    wing.chords.tip              = 0.95  * Units.meter
    wing.chords.mean_aerodynamic = 4.0   * Units.meter
    wing.areas.reference         = 27.316 * Units['meters**2']  
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [28.79,0,1.54] # meters
    wing.vertical                = True 
    wing.symmetric               = False
    # The t tail flag is used in weights calculations
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
        
    # Add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'
    
    # Number of coach seats is used in some weights methods
    fuselage.number_coach_seats    = vehicle.passengers
    # The seats abreast can be used along with seat pitch and the number of coach seats to
    # determine the length of the cabin if desired.
    fuselage.seats_abreast         = 6
    fuselage.seat_pitch            = 1     * Units.meter
    # Fineness ratios are used to determine VLM fuselage shape and sections to use in OpenVSP
    # output
    fuselage.fineness.nose         = 1.6
    fuselage.fineness.tail         = 2.
    # Nose and tail lengths are used in the VLM setup
    fuselage.lengths.nose          = 6.4   * Units.meter
    fuselage.lengths.tail          = 8.0   * Units.meter
    fuselage.lengths.total         = 38.02 * Units.meter
    # Fore and aft space are added to the cabin length if the fuselage is sized based on
    # number of seats
    fuselage.lengths.fore_space    = 6.    * Units.meter
    fuselage.lengths.aft_space     = 5.    * Units.meter
    fuselage.width                 = 3.74  * Units.meter
    fuselage.heights.maximum       = 3.74  * Units.meter
    fuselage.effective_diameter    = 3.74     * Units.meter
    fuselage.areas.side_projected  = 142.1948 * Units['meters**2'] 
    fuselage.areas.wetted          = 446.718  * Units['meters**2'] 
    fuselage.areas.front_projected = 12.57    * Units['meters**2'] 
    # Maximum differential pressure between the cabin and the atmosphere
    fuselage.differential_pressure = 5.0e4 * Units.pascal
    
    # Heights at different longitudinal locations are used in stability calculations and
    # in output to OpenVSP
    fuselage.heights.at_quarter_length          = 3.74 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.65 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.74 * Units.meter
    
    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Turbofan Network
    # ------------------------------------------------------------------    
    
    turbofan = SUAVE.Components.Energy.Networks.Turbofan()
    # For some methods, the 'turbofan' tag is still necessary. This will be changed in the
    # future to allow arbitrary tags.
    turbofan.tag = 'turbofan'
    
    # High-level setup
    turbofan.number_of_engines = 2
    turbofan.bypass_ratio      = 5.4
    turbofan.engine_length     = 2.71 * Units.meter
    turbofan.nacelle_diameter  = 2.05 * Units.meter
    turbofan.origin            = [[13.72, 4.86,-1.9],[13.72, -4.86,-1.9]] # meters
    
    # Approximate the wetted area
    turbofan.areas.wetted      = 1.1*np.pi*turbofan.nacelle_diameter*turbofan.engine_length
    
    # Establish the correct working fluid
    turbofan.working_fluid = SUAVE.Attributes.Gases.Air()
    
    
    # Components use estimated efficiencies. Estimates by technology level can be
    # found in textbooks such as those by J.D. Mattingly
    
    # ------------------------------------------------------------------
    #   Component 1 - Ram
    
    # Converts freestream static to stagnation quantities
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'
    
    # add to the network
    turbofan.append(ram)

    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle
    
    # Create component
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'
    
    # Specify performance
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98
    
    # Add to network
    turbofan.append(inlet_nozzle)
    
    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor
    
    # Create component
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'low_pressure_compressor'

    # Specify performance
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 1.14    
    
    # Add to network
    turbofan.append(compressor)
    
    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    # Create component
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'high_pressure_compressor'
    
    # Specify performance
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 13.415    
    
    # Add to network
    turbofan.append(compressor)

    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine
    
    # Create component
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='low_pressure_turbine'
    
    # Specify performance
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # Add to network
    turbofan.append(turbine)
      
    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine
    
    # Create component
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='high_pressure_turbine'

    # Specify performance
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # Add to network
    turbofan.append(turbine)  
    
    # ------------------------------------------------------------------
    #  Component 7 - Combustor
    
    # Create component    
    combustor = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag = 'combustor'
    
    # Specify performance
    combustor.efficiency                = 0.99 
    combustor.turbine_inlet_temperature = 1450 # K
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    
    
    # Add to network
    turbofan.append(combustor)

    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle
    
    # Create component
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'core_nozzle'
    
    # Specify performance
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    
    # Add to network
    turbofan.append(nozzle)

    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle
    
    # Create component
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'fan_nozzle'

    # Specify performance
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    
    # Add to network
    turbofan.append(nozzle)
    
    # ------------------------------------------------------------------
    #  Component 10 - Fan
    
    # Create component
    fan = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag = 'fan'

    # Specify performance
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7    
    
    # Add to network
    turbofan.append(fan)
    
    # ------------------------------------------------------------------
    #Component 10 : thrust (to compute the thrust)
    thrust = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'
 
    # Design thrust is used to determine mass flow at full throttle
    thrust.total_design             = 2*24000. * Units.N #Newtons
 
    # Design sizing conditions are also used to determine mass flow
    altitude      = 35000.0*Units.ft
    mach_number   = 0.78 
    
    # Add to network
    turbofan.thrust = thrust

    # Determine turbofan behavior at the design condition
    turbofan_sizing(turbofan,mach_number,altitude)   
    
    # Add turbofan network to the vehicle 
    vehicle.append_component(turbofan)      
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    # A max lift coefficient factor of 1 is the default, but it is highlighted here as an option
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].control_surfaces.flap.deflection = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg  
    config.max_lift_coefficient_factor    = 1. 

    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1. 
  
    configs.append(config)

    return configs

def simple_sizing(configs):
    """This function applies a few basic geometric sizing relations and modifies the landing
    configuration."""

    base = configs.base
    # Update the baseline data structure to prepare for changes
    base.pull_base()

    # Revise the zero fuel weight. This will only affect the base configuration. To do all
    # configurations, this should be specified in the top level vehicle definition.
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # Estimate wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # Store how the changes compare to the baseline configuration
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # Make sure base data is current
    landing.pull_base()

    # Add a landing weight parameter. This is used in field length estimation and in
    # initially the landing mission segment type.
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # Store how the changes compare to the baseline configuration
    landing.store_diff()

    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # Airport
    # The airport parameters are used in calculating field length and noise. They are not
    # directly used in mission performance estimation
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # Unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # Base segment 
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    # A constant speed, constant rate climb segment is used first. This means that the aircraft
    # will maintain a constant airspeed and constant climb rate until it hits the end altitude.
    # For this type of segment, the throttle is allowed to vary as needed to match required
    # performance.
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    # It is important that all segment tags must be unique for proper evaluation. At the moment 
    # this is not automatically enforced. 
    segment.tag = "climb_1"

    # The analysis settings for mission segment are chosen here. These analyses include information
    # on the vehicle configuration.
    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 125.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']

    # Add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.cruise )

    # A starting altitude is no longer needed as it will automatically carry over from the
    # previous segment. However, it could be specified if desired. This would potentially cause
    # a jump in altitude but would otherwise not cause any problems.
    segment.altitude_end   = 8.0   * Units.km
    segment.air_speed      = 190.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 10.668 * Units.km
    segment.air_speed    = 226.0  * Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.air_speed  = 230.412 * Units['m/s']
    segment.distance   = 2490. * Units.nautical_miles

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end = 8.0   * Units.km
    segment.air_speed    = 220.0 * Units['m/s']
    segment.descent_rate = 4.5   * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.landing )

    segment.altitude_end = 6.0   * Units.km
    segment.air_speed    = 195.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend( analyses.landing )
    # While it is set to zero here and therefore unchanged, a drag increment can be used if
    # desired. This can avoid negative throttle values if drag generated by the base airframe
    # is insufficient for the desired descent speed and rate.
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    segment.altitude_end = 4.0   * Units.km
    segment.air_speed    = 170.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4"

    segment.analyses.extend( analyses.landing )
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    segment.altitude_end = 2.0   * Units.km
    segment.air_speed    = 150.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fifth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5"

    segment.analyses.extend( analyses.landing )
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 145.0 * Units['m/s']
    segment.descent_rate = 3.0   * Units['m/s']

    # Append to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    # Setup the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    # Only one mission (the base mission) is defined in this case
    missions.base = base_mission

    return missions  

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    """This function plots the results of the mission analysis and saves those results to
    PDF and png files."""

    axis_font = {'fontname':'Arial', 'size':'14'}    

    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------


    fig = plt.figure("Aerodynamic Forces",figsize=(8,6))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lbf
        eta    = segment.conditions.propulsion.throttle[:,0]

        axes = fig.add_subplot(2,1,1)
        axes.plot( time , Thrust , line_style )
        axes.set_ylabel('Thrust (lbf)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(2,1,2)
        axes.plot( time , eta , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Throttle',axis_font)
        axes.grid(True)	

        plt.savefig("B737_engine.pdf")
        plt.savefig("B737_engine.png")

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        aoa = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d = CLift/CDrag

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , line_style )
        axes.set_ylabel('Lift Coefficient',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , l_d , line_style )
        axes.set_ylabel('L/D',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , aoa , 'ro-' )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('AOA (deg)',axis_font)
        axes.grid(True)

        plt.savefig("B737_aero.pdf")
        plt.savefig("B737_aero.png")

    # ------------------------------------------------------------------
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components",figsize=(8,10))
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]

        if line_style == 'bo-':
            axes.plot( time , cdp , 'ko-', label='CD parasite' )
            axes.plot( time , cdi , 'bo-', label='CD induced' )
            axes.plot( time , cdc , 'go-', label='CD compressibility' )
            axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
            axes.plot( time , cd  , 'ro-', label='CD total'   )
            if i == 0:
                axes.legend(loc='upper center')            
        else:
            axes.plot( time , cdp , line_style )
            axes.plot( time , cdi , line_style )
            axes.plot( time , cdc , line_style )
            axes.plot( time , cdm , line_style )
            axes.plot( time , cd  , line_style )            

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    plt.savefig("B737_drag.pdf")
    plt.savefig("B737_drag.png")

    # ------------------------------------------------------------------
    #   Altitude, sfc, vehicle weight
    # ------------------------------------------------------------------

    fig = plt.figure("Altitude_sfc_weight",figsize=(8,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        mass     = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        mdot     = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust   =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc      = (mdot / Units.lb) / (thrust /Units.lbf) * Units.hr

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_style )
        axes.set_ylabel('Altitude (ft)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , sfc , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('sfc (lb/lbf-hr)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        axes.grid(True)

        plt.savefig("B737_mission.pdf")
        plt.savefig("B737_mission.png")
        
    # ------------------------------------------------------------------
    #   Velocities
    # ------------------------------------------------------------------
    fig = plt.figure("Velocities",figsize=(8,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift     = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag     = -segment.conditions.frames.wind.drag_force_vector[:,0] / Units.lbf
        Thrust   = segment.conditions.frames.body.thrust_force_vector[:,0] / Units.lb
        velocity = segment.conditions.freestream.velocity[:,0]
        pressure = segment.conditions.freestream.pressure[:,0]
        density  = segment.conditions.freestream.density[:,0]
        EAS      = velocity * np.sqrt(density/1.225)
        mach     = segment.conditions.freestream.mach_number[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , velocity / Units.kts, line_style )
        axes.set_ylabel('velocity (kts)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , EAS / Units.kts, line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Equivalent Airspeed',axis_font)
        axes.grid(True)    
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach',axis_font)
        axes.grid(True)           
        
    return

# This section is needed to actually run the various functions in the file
if __name__ == '__main__': 
    main()    
    # The show commands makes the plots actually appear
    plt.show()