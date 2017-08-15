# Vehicles.py
# 
# Created:  Feb 2016, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import SUAVE
import numpy as np
from SUAVE.Core import Units, Data
from SUAVE.Components.Energy.Networks.Solar_Low_Fidelity import Solar_Low_Fidelity
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv

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
    vehicle.tag = 'Solar'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff         = 6.75 * Units.kg
    vehicle.mass_properties.operating_empty = 6.75 * Units.kg
    vehicle.mass_properties.max_takeoff     = 6.75 * Units.kg 

    # basic parameters
    vehicle.reference_area                    = 1.0       
    vehicle.envelope.ultimate_load            = 2.0
    vehicle.envelope.limit_load               = 1.5
    vehicle.envelope.maximum_dynamic_pressure = 115.*1.25 * Units.pascals #Max q

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'main_wing'

    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 40.0 * Units.meters
    wing.aspect_ratio            = (wing.spans.projected**2)/wing.areas.reference 
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.symmetric               = True
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.vertical                = False
    wing.high_lift               = True 
    wing.dynamic_pressure_ratio  = 1.0
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.span_efficiency         = 0.98 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.highlift                = False  
    wing.vertical                = False 
    wing.number_ribs             = 26.
    wing.number_end_ribs         = 2.
    wing.transition_x_upper      = 0.6
    wing.transition_x_lower      = 1.0
    wing.origin                  = [3.0,0.0,0.0]

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio         = 10. 
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord   = 0.12
    wing.taper                = 1.0
    wing.span_efficiency      = 0.95 
    wing.areas.reference      = vehicle.reference_area * .15
    wing.areas.wetted         = 2.0 * wing.areas.reference
    wing.areas.exposed        = 0.8 * wing.areas.wetted
    wing.areas.affected       = 0.6 * wing.areas.wetted       
    wing.spans.projected      = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.twists.root          = 0.0 * Units.degrees
    wing.twists.tip           = 0.0 * Units.degrees      

    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9      
    wing.number_ribs             = 5.0
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected  
    wing.origin                  = [10.,0.0,0.0]

    # add to vehicle
    vehicle.append_component(wing)    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio         = 10.       
    wing.sweeps.quarter_chord = 0 * Units.deg
    wing.thickness_to_chord   = 0.12
    wing.taper                = 1.0
    wing.span_efficiency      = 0.97
    wing.areas.reference      = vehicle.reference_area * 0.1
    wing.spans.projected      = np.sqrt(wing.aspect_ratio*wing.areas.reference)

    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [10.,0.0,0.0]      
    wing.symmetric               = True          
    wing.vertical                = True 
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
    wing.number_ribs             = 5.

    # add to vehicle
    vehicle.append_component(wing)  

    #------------------------------------------------------------------
    # Propulsor
    #------------------------------------------------------------------

    # build network
    net = Solar_Low_Fidelity()
    net.number_of_engines = 1.
    net.nacelle_diameter  = 0.05
    net.areas             = Data()
    net.areas.wetted      = 0.01*(2*np.pi*0.01/2)
    net.engine_length     = 0.01

    # Component 1 the Sun
    sun = SUAVE.Components.Energy.Processes.Solar_Radiation()
    net.solar_flux = sun

    # Component 2 the solar panels
    panel = SUAVE.Components.Energy.Converters.Solar_Panel()
    panel.ratio                = 0.9
    panel.area                 = vehicle.reference_area * panel.ratio 
    panel.efficiency           = 0.25
    panel.mass_properties.mass = panel.area*(0.60 * Units.kg)
    net.solar_panel            = panel

    # Component 3 the ESC
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc        = esc

    # Component 5 the Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
    prop.propulsive_efficiency = 0.825
    net.propeller        = prop
    
    # Component 4 the Motor
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    kv                         = 800. * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, kv)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = 1. # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor                  = motor    

    # Component 6 the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 0. #Watts 
    payload.mass_properties.mass = 0.0 * Units.kg
    net.payload                  = payload

    # Component 7 the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 10. #Watts  
    net.avionics        = avionics      

    # Component 8 the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 5.0  * Units.kg
    bat.specific_energy      = 250. *Units.Wh/Units.kg
    bat.resistance           = 0.003
    bat.iters                = 0
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat

    #Component 9 the system logic controller and MPPT
    logic = SUAVE.Components.Energy.Distributors.Solar_Logic()
    logic.system_voltage  = 18.5
    logic.MPPT_efficiency = 0.95
    net.solar_logic       = logic

    # add the solar network to the vehicle
    vehicle.append_component(net)  

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Only One Configurations
    # ------------------------------------------------------------------

    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs