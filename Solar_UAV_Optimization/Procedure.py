# Procedure.py
# 
# Created:  Feb 2016, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

from SUAVE.Core import Units
from SUAVE.Analyses.Process import Process
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_mass
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing   = simple_sizing
    
    # Size the battery and charge it before the mission
    procedure.weights_battery = weights_battery

    # finalizes the data dependencies
    procedure.finalize        = finalize
    
    # performance studies
    procedure.missions        = Process()
    procedure.missions.base   = simple_mission
    
    # Post process the results
    procedure.post_process    = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Simple Mission
# ----------------------------------------------------------------------    
    
def simple_mission(nexus):
    
    mission = nexus.missions.mission

    # Evaluate the missions and save to results   
    results         = nexus.results
    results.mission = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):
    
    # Pull out the vehicle
    vec = nexus.vehicle_configurations.base
    
    # Change the dynamic pressure based on the, add a factor of safety   
    vec.envelope.maximum_dynamic_pressure = nexus.missions.mission.segments.cruise.dynamic_pressure*1.2
    
    # Scale the horizontal and vertical tails based on the main wing area
    vec.wings.horizontal_stabilizer.areas.reference = 0.15 * vec.reference_area
    vec.wings.vertical_stabilizer.areas.reference   = 0.08 * vec.reference_area

    # wing spans,areas, and chords
    for wing in vec.wings:
        
        # Unpack
        AR = wing.aspect_ratio
        S  = wing.areas.reference
        
        # Set the spans
        wing.spans.projected = np.sqrt(AR*S)
        
        # Set all of the areas for the surfaces
        wing.areas.wetted   = 2.0 * S
        wing.areas.exposed  = 1.0 * wing.areas.wetted
        wing.areas.affected = 1.0 * wing.areas.wetted   
        
        # Set all of the chord lengths
        chord = wing.areas.reference/wing.spans.projected
        wing.chords.mean_aerodynamic = chord
        wing.chords.mean_geometric   = chord
        wing.chords.root             = chord
        wing.chords.tip              = chord

    # Size solar panel area
    wing_area                   = vec.reference_area
    spanel                      = vec.propulsors.network.solar_panel
    sratio                      = spanel.ratio
    solar_area                  = wing_area*sratio
    spanel.area                 = solar_area
    spanel.mass_properties.mass = solar_area*(0.60 * Units.kg)    
    
    # Resize the motor
    motor = vec.propulsors.network.motor
    kv    = motor.speed_constant
    motor = size_from_kv(motor, kv)    
    
    # diff the new data
    vec.store_diff()

    return nexus

# ----------------------------------------------------------------------
#   Calculate weights and charge the battery
# ---------------------------------------------------------------------- 

def weights_battery(nexus):

    # Evaluate weights for all of the configurations
    config = nexus.analyses.base
    config.weights.evaluate() 
    
    vec     = nexus.vehicle_configurations.base
    payload = vec.propulsors.network.payload.mass_properties.mass  
    msolar  = vec.propulsors.network.solar_panel.mass_properties.mass
    MTOW    = vec.mass_properties.max_takeoff
    empty   = vec.weight_breakdown.empty
    mmotor  = vec.propulsors.network.motor.mass_properties.mass
    
    # Calculate battery mass
    batmass = MTOW - empty - payload - msolar -mmotor
    bat     = vec.propulsors.network.battery
    initialize_from_mass(bat,batmass)
    vec.propulsors.network.battery.mass_properties.mass = batmass
        
    # Set Battery Charge
    maxcharge = nexus.vehicle_configurations.base.propulsors.network.battery.max_energy
    charge    = maxcharge
    
    nexus.missions.mission.segments.cruise.battery_energy = charge 

    return nexus
    
# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
    return nexus         

# ----------------------------------------------------------------------
#   Post Process results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack
    mis = nexus.missions.mission.segments.cruise
    vec = nexus.vehicle_configurations.base
    res = nexus.results.mission.segments.cruise.conditions
    
    # Final Energy
    maxcharge    = vec.propulsors.network.battery.max_energy
    
    # Energy constraints, the battery doesn't go to zero anywhere, using a P norm
    p                    = 8.    
    energies             = res.propulsion.battery_energy[:,0]/np.abs(maxcharge)
    energies[energies>0] = 0.0 # Exclude the values greater than zero
    energy_constraint    = np.sum((np.abs(energies)**p))**(1/p) 

    # CL max constraint, it is the same throughout the mission
    CL = res.aerodynamics.lift_coefficient[0]
    
    # Pack up
    summary = nexus.summary
    summary.CL                = 1.2 - CL
    summary.energy_constraint = energy_constraint
    summary.throttle_min      = res.propulsion.throttle[0]
    summary.throttle_max      = 0.9 - res.propulsion.throttle[0]
    summary.nothing           = 0.0
    
    return nexus    