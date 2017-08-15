# Vehicles.py
# 
# Created:  Feb. 2016, M. Vegh
# Modified: Aug. 2017, E. Botero

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
from SUAVE.Core import Units
from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Embraer_E190'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff               = 52000  * Units.kg
    vehicle.mass_properties.operating_empty           = 27837. * Units.kg
    vehicle.mass_properties.takeoff                   = 52000  * Units.kg
    vehicle.mass_properties.max_zero_fuel             = 42977. * Units.kg
    vehicle.mass_properties.cargo                     = 0.0    * Units.kg
    vehicle.mass_properties.max_payload               = 13063. * Units.kg
    vehicle.mass_properties.max_fuel                  = 12971. * Units.kg

    vehicle.mass_properties.center_of_gravity         = [18., 0, 0]
    #vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] # Not Correct

    # envelope properties
    vehicle.envelope.ultimate_load = 3.5
    vehicle.envelope.limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 92.00 * Units['meters**2']  
    vehicle.passengers             = 114
    vehicle.systems.control        = "fully powered"
    vehicle.systems.accessories    = "medium range"

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 8.4
    wing.sweeps.quarter_chord    = 23.0 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.28
    wing.span_efficiency         = 1.0
    wing.spans.projected         = 27.8  * Units.meter
    wing.chords.root             = 5.203 * Units.meter
    wing.chords.tip              = 1.460 * Units.meter
    wing.chords.mean_aerodynamic = 3.680 * Units.meter
    wing.areas.reference         = 92.0  * Units['meters**2']  
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.areas.exposed           = 0.8   * wing.areas.wetted
    wing.areas.affected          = 0.6   * wing.areas.reference
    wing.twists.root             = 2.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [13.2,0,0]
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.flaps.type              = "double_slotted"
    wing.flaps.chord             = 0.280 * Units.meter
    wing.dynamic_pressure_ratio  = 1.0

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 5.5
    wing.sweeps.quarter_chord    = 34.5 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.11
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 11.958 * Units.meter
    wing.chords.root             = 3.030  * Units.meter
    wing.chords.tip              = 0.883  * Units.meter
    wing.chords.mean_aerodynamic = 2.3840 * Units.meter
    wing.areas.reference         = 26.0 * Units['meters**2'] 
    wing.areas.wetted            = 2.0  * wing.areas.reference
    wing.areas.exposed           = 0.8  * wing.areas.wetted
    wing.areas.affected          = 0.6  * wing.areas.reference
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 2.0 * Units.degrees
    wing.origin                  = [31.,0,0]
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

    wing.aspect_ratio            = 1.7   
    wing.sweeps.quarter_chord    = 35 * Units.deg
    wing.thickness_to_chord      = 0.11
    wing.taper                   = 0.31
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 5.270 * Units.meter
    wing.chords.root             = 4.70  * Units.meter
    wing.chords.tip              = 1.45  * Units.meter
    wing.chords.mean_aerodynamic = 3.36  * Units.meter
    wing.areas.reference         = 16.0  * Units['meters**2'] 
    wing.areas.wetted            = 2.0   * wing.areas.reference
    wing.areas.exposed           = 0.8   * wing.areas.wetted
    wing.areas.affected          = 0.6   * wing.areas.reference
    wing.twists.root             = 0.0   * Units.degrees
    wing.twists.tip              = 0.0   * Units.degrees
    wing.origin                  = [29.5,0,0]
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
    fuselage.fineness.nose         = 2.0
    fuselage.fineness.tail         = 3.0
    fuselage.lengths.nose          = 6.0   * Units.meter
    fuselage.lengths.tail          = 9.0   * Units.meter
    fuselage.lengths.cabin         = 21.24 * Units.meter
    fuselage.lengths.total         = 36.24 * Units.meter
    fuselage.lengths.fore_space    = 0.    * Units.meter
    fuselage.lengths.aft_space     = 0.    * Units.meter
    fuselage.width                 = 3.18  * Units.meter
    fuselage.heights.maximum       = 3.50  * Units.meter
    fuselage.areas.side_projected  = 239.20 * Units['meters**2'] 
    fuselage.areas.wetted          = 327.01 * Units['meters**2'] 
    fuselage.areas.front_projected = 8.0110 * Units['meters**2']    
    fuselage.effective_diameter    = 3.18
    fuselage.differential_pressure = 10**5 * Units.pascal 
    
    fuselage.heights.at_quarter_length          = 3.35 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.35 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.50 * Units.meter

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #  Turbofan Network
    # ------------------------------------------------------------------    

    #initialize the gas turbine network
    gt_engine                   = SUAVE.Components.Energy.Networks.Turbofan()
    gt_engine.tag               = 'turbofan'

    gt_engine.number_of_engines = 2.0
    gt_engine.bypass_ratio      = 5.4
    gt_engine.engine_length     = 2.71
    gt_engine.nacelle_diameter  = 2.05

    #set the working fluid for the network
    gt_engine.working_fluid = SUAVE.Attributes.Gases.Air


    #Component 1 : ram,  to convert freestream static to stagnation quantities
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'

    #add ram to the network
    gt_engine.ram = ram

    #Component 2 : inlet nozzle
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet nozzle'

    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98

    #add inlet nozzle to the network
    gt_engine.inlet_nozzle = inlet_nozzle

    #Component 3 :low pressure compressor    
    low_pressure_compressor = SUAVE.Components.Energy.Converters.Compressor()    
    low_pressure_compressor.tag = 'lpc'

    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9    

    #add low pressure compressor to the network    
    gt_engine.low_pressure_compressor = low_pressure_compressor

    #Component 4: high pressure compressor  
    high_pressure_compressor = SUAVE.Components.Energy.Converters.Compressor()    
    high_pressure_compressor.tag = 'hpc'

    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.0   

    #add the high pressure compressor to the network    
    gt_engine.high_pressure_compressor = high_pressure_compressor

    #Component 5 :low pressure turbine  
    low_pressure_turbine = SUAVE.Components.Energy.Converters.Turbine()   
    low_pressure_turbine.tag='lpt'

    low_pressure_turbine.mechanical_efficiency = 0.99
    low_pressure_turbine.polytropic_efficiency = 0.93

    #add low pressure turbine to the network    
    gt_engine.low_pressure_turbine = low_pressure_turbine

    #Component 5 :high pressure turbine  
    high_pressure_turbine = SUAVE.Components.Energy.Converters.Turbine()   
    high_pressure_turbine.tag='hpt'

    high_pressure_turbine.mechanical_efficiency = 0.99
    high_pressure_turbine.polytropic_efficiency = 0.93

    #add the high pressure turbine to the network    
    gt_engine.high_pressure_turbine = high_pressure_turbine 

    #Component 6 :combustor  
    combustor = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag = 'Comb'

    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0     
    combustor.turbine_inlet_temperature = 1500
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    

    #add the combustor to the network    
    gt_engine.combustor = combustor

    #Component 7 :core nozzle
    core_nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    core_nozzle.tag = 'core nozzle'

    core_nozzle.polytropic_efficiency = 0.95
    core_nozzle.pressure_ratio        = 0.99    

    #add the core nozzle to the network    
    gt_engine.core_nozzle = core_nozzle

    #Component 8 :fan nozzle
    fan_nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    fan_nozzle.tag = 'fan nozzle'

    fan_nozzle.polytropic_efficiency = 0.95
    fan_nozzle.pressure_ratio        = 0.99

    #add the fan nozzle to the network
    gt_engine.fan_nozzle = fan_nozzle

    #Component 9 : fan   
    fan = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag = 'fan'

    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7    

    #add the fan to the network
    gt_engine.fan = fan    

    #Component 10 : thrust (to compute the thrust)
    thrust = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'

    #total design thrust (includes all the engines)
    thrust.total_design             = 52700.0* Units.N #Newtons
                                        
    #design sizing conditions
    altitude      = 35000.0*Units.ft
    mach_number   = 0.78 
    isa_deviation = 0.

    # add thrust to the network
    gt_engine.thrust = thrust

    #size the turbofan
    turbofan_sizing(gt_engine,mach_number,altitude)   

    # add  gas turbine network gt_engine to the vehicle
    vehicle.append_component(gt_engine)      

    #now add weights objects
    vehicle.landing_gear       = SUAVE.Components.Landing_Gear.Landing_Gear()
    vehicle.control_systems    = SUAVE.Components.Physical_Component()
    vehicle.electrical_systems = SUAVE.Components.Physical_Component()
    vehicle.avionics           = SUAVE.Components.Energy.Peripherals.Avionics()
    vehicle.passenger_weights  = SUAVE.Components.Physical_Component()
    vehicle.furnishings        = SUAVE.Components.Physical_Component()
    vehicle.air_conditioner    = SUAVE.Components.Physical_Component()
    vehicle.fuel               = SUAVE.Components.Physical_Component()
    vehicle.apu                = SUAVE.Components.Physical_Component()
    vehicle.hydraulics         = SUAVE.Components.Physical_Component()
    vehicle.optionals          = SUAVE.Components.Physical_Component()

    vehicle.wings['vertical_stabilizer'].rudder = SUAVE.Components.Physical_Component()
    
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
    
    config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Cruise with Spoilers Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise_spoilers'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2.

    configs.append(config)

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2. 
    
    configs.append(config)

    return configs
