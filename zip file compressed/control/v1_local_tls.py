import traci
import traci.constants as tc
import os
import sys
import time

# ---------------- CONFIG ----------------
SUMO_CFG = "config/grid.sumocfg"

MIN_GREEN = 5      # seconds
MAX_GREEN = 30     # seconds
CONTROL_STEP = 5   # seconds
# ----------------------------------------

def get_queue_length(lanes):
    """Return total halting vehicles on given lanes"""
    q = 0
    for lane in lanes:
        q += traci.lane.getLastStepHaltingNumber(lane)
    return q


def main():
    sumo_cmd = [
        "sumo-gui",
        "-c", SUMO_CFG,
        "--step-length", "1"
    ]

    traci.start(sumo_cmd)
    print("SUMO started with V1 local adaptive control")

    tls_ids = traci.trafficlight.getIDList()
    print("Traffic Lights:", tls_ids)

    # Track timing
    last_switch = {tls: 0 for tls in tls_ids}
    green_start = {tls: 0 for tls in tls_ids}

    sim_time = 0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        sim_time = traci.simulation.getTime()

        if sim_time % CONTROL_STEP != 0:
            continue

        for tls in tls_ids:
            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
            phases = logic.phases
            current_phase = traci.trafficlight.getPhase(tls)

            # Enforce min / max green
            elapsed = sim_time - green_start[tls]
            if elapsed < MIN_GREEN:
                continue

            # Collect pressure per phase
            phase_pressure = []

            controlled_lanes = traci.trafficlight.getControlledLanes(tls)

            for i, phase in enumerate(phases):
                lanes_for_phase = []
                for idx, state in enumerate(phase.state):
                    if state == 'G':
                        lanes_for_phase.append(controlled_lanes[idx])

                pressure = get_queue_length(set(lanes_for_phase))
                phase_pressure.append(pressure)

            best_phase = phase_pressure.index(max(phase_pressure))

            # Switch if:
            # - different phase
            # - OR max green exceeded
            if best_phase != current_phase or elapsed >= MAX_GREEN:
                traci.trafficlight.setPhase(tls, best_phase)
                green_start[tls] = sim_time
                last_switch[tls] = sim_time

        # Optional live print every 50s
        if sim_time % 50 == 0:
            print(f"t={sim_time}s running vehicles:",
                  traci.simulation.getMinExpectedNumber())

    traci.close()
    print("Simulation finished (V1)")


if __name__ == "__main__":
    main()
