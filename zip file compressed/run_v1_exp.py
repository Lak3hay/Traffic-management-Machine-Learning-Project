import traci
import time
import csv
import statistics

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"
SUMOCFG = "config/grid.sumocfg" 

CONTROL_INTERVAL = 60
MIN_GREEN = 15
MAX_GREEN = 60
MAX_SIM_TIME = 2000
# ---------------------------------------


def lane_queue(lane_id):
    return traci.lane.getLastStepHaltingNumber(lane_id)


def main():
    traci.start([SUMO_BINARY, "-c", SUMOCFG, "--start"])
    print("V1 Fixed-Time controller started")

    tls_ids = traci.trafficlight.getIDList()
    last_switch = {tls: 0 for tls in tls_ids}

    # ---------- PERFORMANCE CSV ----------
    perf_log = open("results_v1_experiment.csv", "w", newline="")
    perf_writer = csv.writer(perf_log)
    perf_writer.writerow(["time", "avg_speed", "running", "halted"])

    # ---------- CONTROL LOG CSV ----------
    ctrl_log = open("control_log_v1_experiment.csv", "w", newline="")
    ctrl_writer = csv.writer(ctrl_log)
    ctrl_writer.writerow([
        "time",
        "tls",
        "selected_phase",
        "max_queue",
        "avg_queue",
        "phase_switched"
    ])
    # ------------------------------------

    step = 0

    while step < MAX_SIM_TIME:
        traci.simulationStep()
        step += 1

        for tls in tls_ids:
            elapsed = step - last_switch[tls]

            if elapsed < CONTROL_INTERVAL or elapsed < MIN_GREEN:
                continue

            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
            num_phases = len(logic.phases)

            current_phase = traci.trafficlight.getPhase(tls)
            next_phase = (current_phase + 1) % num_phases

            # fixed-time switch
            traci.trafficlight.setPhase(tls, next_phase)
            last_switch[tls] = step

            # ---- CONTROL LOGGING ----
            queues = [
                lane_queue(l)
                for lg in traci.trafficlight.getControlledLinks(tls)
                for l, _, _ in lg
            ]

            ctrl_writer.writerow([
                step,
                tls,
                next_phase,
                max(queues) if queues else 0,
                statistics.mean(queues) if queues else 0,
                1  # always switches in fixed-time
            ])
            # -------------------------

        # ---------- METRICS ----------
        veh_ids = traci.vehicle.getIDList()
        running = len(veh_ids)

        if running > 0:
            avg_speed = sum(traci.vehicle.getSpeed(v) for v in veh_ids) / running
            halted = sum(1 for v in veh_ids if traci.vehicle.getSpeed(v) < 0.1)
        else:
            avg_speed = 0.0
            halted = 0

        perf_writer.writerow([step, avg_speed, running, halted])
        # -----------------------------

        if traci.simulation.getMinExpectedNumber() == 0:
            break

    perf_log.close()
    ctrl_log.close()
    traci.close()
    print("V1 experimental Simulation ended")


if __name__ == "__main__":
    main()
