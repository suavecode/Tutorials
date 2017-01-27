# Vehicles.py
# 
# Created:  Jan 2017, M. Vegh
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units, Data
from SUAVE.Methods.Propulsion.ducted_fan_sizing import ducted_fan_sizing

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs
    
    
def base_setup():
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Embraer_E190'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------
    '''
    # mass properties
    vehicle.mass_properties.max_takeoff               = 92110. #use landing mass as 
    vehicle.mass_properties.operating_empty           = 34551. 
    vehicle.mass_properties.takeoff                   = 80721. 
    vehicle.mass_properties.max_zero_fuel             = 92110. #equivalent landing mass
    vehicle.mass_properties.cargo                     = 0.0 
    vehicle.mass_properties.max_payload               = 0.0 
    vehicle.mass_properties.max_fuel                  = 0.0
    '''
    
    # envelope properties
    vehicle.envelope.ultimate_load = 3.5
    vehicle.envelope.limit_load    = 1.5
    
    # basic parameters
    vehicle.reference_area         = 100.0
    vehicle.thrust_loading         =.2
    vehicle.wing_loading           = 400.*Units.kg/Units.m**2
    vehicle.passengers             = 114
    vehicle.systems.control        = "partially powered"
    vehicle.systems.accessories    = "medium range"
    vehicle.w2h                    = 16.     * Units.meters    # Length from the mean aerodynamic center of wing to mean aerodynamic center of the horizontal tail
    vehicle.w2v                    = 20.     * Units.meters    # Length from the mean aerodynamic center of wing to mean aerodynamic center of the vertical tail
    vehicle.motor_power            = 80      * Units.MW
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 8.3
    wing.sweeps.quarter_chord    = 0.002 * Units.deg
    wing.thickness_to_chord      = 0.105
    wing.taper                   = 0.28
    wing.span_efficiency         = 1.0

    wing.spans.projected         = 28.81

    wing.chords.root             = 5.4
    wing.chords.tip              = 1.5185
    wing.chords.mean_aerodynamic = 3.8371

    wing.areas.reference         = 100.0
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 150.0
    wing.areas.affected          = .6*wing.areas.reference

    wing.twists.root             = -0.311 * Units.degrees
    wing.twists.tip              = -0.315 * Units.degrees

    wing.origin                  = [00,0,0]
    wing.aerodynamic_center      = [0.914,0,0]

    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    
    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 5.5
    wing.sweeps.quarter_chord   = 0.002 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.11
    wing.span_efficiency         = 0.9

    wing.spans.projected         = 11.213

    wing.chords.root             = 3.6733
    wing.chords.tip              = 0.4040
    wing.chords.mean_aerodynamic = 2.4755

    wing.areas.reference         = 22.85
    wing.areas.wetted            = 45.71
    wing.areas.exposed           = 34.28
    wing.areas.affected          = 13.71

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [50,0,0]
    wing.aerodynamic_center      = [16.9144, 0.0, 0.0]

    wing.vertical                = False
    wing.symmetric               = True

    wing.dynamic_pressure_ratio  = 0.9

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 1.7      #
    wing.sweeps.quarter_chord    = 0. * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.10
    wing.span_efficiency         = 0.9

    wing.spans.projected         = 4.6945     #

    wing.chords.root             = 5.0209
    wing.chords.tip              = 0.5020
    wing.chords.mean_aerodynamic = 3.3777

    wing.areas.reference         = 12.96
    wing.areas.wetted            = 25.92
    wing.areas.exposed           = 19.44
    wing.areas.affected          = 7.778

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [0,0,0]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.symmetric               = False

    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 4
    fuselage.seat_pitch            = 0.7455

    fuselage.fineness.nose         = 1.5
    fuselage.fineness.tail         = 1.8

    fuselage.lengths.nose          = 4.5
    fuselage.lengths.tail          = 5.4
    fuselage.lengths.cabin         = 21.24675
    fuselage.lengths.total         = 31.14675
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.

    fuselage.width                 = 3.0

    fuselage.heights.maximum       = 3.4    #
    fuselage.heights.at_quarter_length          = 3.4
    fuselage.heights.at_three_quarters_length   = 3.4
    fuselage.heights.at_wing_root_quarter_chord = 3.4

    fuselage.areas.side_projected  = 79.462
    fuselage.areas.wetted          = 288.521422366
    fuselage.areas.front_projected = 8.011

    fuselage.effective_diameter    = 3.2

    fuselage.differential_pressure = 10**5 * Units.pascal 

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #  Propulsion

     
    #create battery
    battery = SUAVE.Components.Energy.Storages.Batteries.Variable_Mass.Lithium_Air()
    battery.tag = 'battery'
   
    #Build Ducted_Fan Network
    ducted_fan= SUAVE.Components.Energy.Networks.Ducted_Fan()
    #set the working fluid for the network
    working_fluid               = SUAVE.Attributes.Gases.Air

    #add working fluid to the network
    ducted_fan.working_fluid    = working_fluid
    
   
    #Component 1 : ram,  to convert freestream static to stagnation quantities
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'

    #add ram to the network
    ducted_fan.ram = ram


    #Component 2 : inlet nozzle
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet nozzle'

    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98 #	turbofan.fan_nozzle_pressure_ratio     = 0.98     #0.98

    #add inlet nozzle to the network
    ducted_fan.inlet_nozzle = inlet_nozzle


    #Component 8 :fan nozzle
    fan_nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    fan_nozzle.tag = 'fan nozzle'

    fan_nozzle.polytropic_efficiency = 0.95
    fan_nozzle.pressure_ratio        = 0.99

    #add the fan nozzle to the network
    ducted_fan.fan_nozzle = fan_nozzle



    #Component 9 : fan   
    fan = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag = 'fan'

    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.5    

    #add the fan to the network
    ducted_fan.fan = fan    

    #Component 10 : thrust (to compute the thrust)
    thrust                    = SUAVE.Components.Energy.Processes.Thrust()  
    thrust.tag                ='compute_thrust'
    thrust.total_design       = 121979.18 # Preq*1.5/V_cruise 
    ducted_fan.thrust         = thrust
    #ducted_fan.design_thrust  =
    
    
    
    atm = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    p1, T1, rho1, a1, mew1 = atm.compute_values(0.)
    p2, T2, rho2, a2, mew2 = atm.compute_values(6.255*Units.km)
  

    mach_number                   =0.729  
    altitude                      = 30000*Units.ft
    
    ducted_fan.number_of_engines  = 2.0    
    ducted_fan_sizing(ducted_fan, mach_number, altitude)   #calling the engine sizing method 
    
    # ------------------------------------------------------------------
    #  Energy Network
    # ------------------------------------------------------------------ 
    
    #define the energy network
    net     = SUAVE.Components.Energy.Networks.Battery_Ducted_Fan()
    net.tag = 'network'
    
    net.propulsor = ducted_fan
    net.append(ducted_fan)
    net.battery = battery
    
    net.nacelle_diameter  = ducted_fan.nacelle_diameter
    net.engine_length     = ducted_fan.engine_length    
    net.number_of_engines = ducted_fan.number_of_engines
    net.motor_efficiency  =.95
    vehicle.propulsors.append(ducted_fan)
    vehicle.energy_network = net
    
    
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

    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio = 1.21
    #config.maximum_lift_coefficient = 2.

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio = 1.23
    #config.maximum_lift_coefficient = 2.

    configs.append(config)


    # done!
    return configs
