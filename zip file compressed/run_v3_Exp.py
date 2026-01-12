import traci
import csv
import statistics

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"
SUMOCFG = "config/grid.sumocfg"

CONTROL_INTERVAL = 10
MAX_SIM_TIME = 2000
# ---------------------------------------


def lane_queue(lane_id):
    return traci.lane.getLastStepHaltingNumber(lane_id)


def compute_phase_pressure(tls_id, phase_index):
    """
    TRUE Max-Pressure:
    Sum over (q_up - q_down) for all GREEN movements
    """
    logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0]
    phase = logic.phases[phase_index]
    links = traci.trafficlight.getControlledLinks(tls_id)

    pressure = 0.0

    for link_group, signal in zip(links, phase.state):
        if signal.lower() != 'g':
            continue

        for in_lane, out_lane, _ in link_group:
            q_up = lane_queue(in_lane)
            q_down = lane_queue(out_lane) if out_lane else 0
            pressure += (q_up - q_down)

    return pressure


def log_metrics(writer, step):
    veh_ids = traci.vehicle.getIDList()
    running = len(veh_ids)

    if running > 0:
        avg_speed = sum(traci.vehicle.getSpeed(v) for v in veh_ids) / running
        halted = sum(1 for v in veh_ids if traci.vehicle.getSpeed(v) < 0.1)
    else:
        avg_speed = 0.0
        halted = 0

    writer.writerow([step, avg_speed, running, halted])
    return avg_speed, running


def main():
    traci.start([
        SUMO_BINARY,
        "-c", SUMOCFG,
        "--start",
        "--quit-on-end"
    ])

    print("V3 TRUE Max-Pressure Simulation started")

    # ---------- PERFORMANCE CSV ----------
    perf_log = open("results_v3_experiment.csv", "w", newline="")
    perf_writer = csv.writer(perf_log)
    perf_writer.writerow(["time", "avg_speed", "running", "halted"])

    # ---------- CONTROL LOG CSV ----------
    ctrl_log = open("control_log_v3_experiment.csv", "w", newline="")
    ctrl_writer = csv.writer(ctrl_log)
    ctrl_writer.writerow([
        "time",
        "tls",
        "selected_phase",
        "pressure_best",
        "pressure_second",
        "pressure_gap",
        "phase_switched"
    ])

    tls_ids = traci.trafficlight.getIDList()
    last_control = {tls: 0 for tls in tls_ids}

    step = 0

    while step < MAX_SIM_TIME:
        traci.simulationStep()
        step += 1

        for tls in tls_ids:
            if step - last_control[tls] < CONTROL_INTERVAL:
                continue

            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
            num_phases = len(logic.phases)

            pressures = [
                compute_phase_pressure(tls, p)
                for p in range(num_phases)
            ]

            best_phase = pressures.index(max(pressures))
            sorted_p = sorted(pressures, reverse=True)

            current_phase = traci.trafficlight.getPhase(tls)
            switched = int(best_phase != current_phase)

            traci.trafficlight.setPhase(tls, best_phase)
            last_control[tls] = step

            # ---- CONTROL LOG ----
            ctrl_writer.writerow([
                step,
                tls,
                best_phase,
                sorted_p[0],
                sorted_p[1] if len(sorted_p) > 1 else 0,
                sorted_p[0] - (sorted_p[1] if len(sorted_p) > 1 else 0),
                switched
            ])

        avg_speed, running = log_metrics(perf_writer, step)

        if step % 20 == 0:
            print(f"t={step:4d}s | avg_speed={avg_speed:5.2f} | running={running}")

        if traci.simulation.getMinExpectedNumber() == 0:
            break

    perf_log.close()
    ctrl_log.close()
    traci.close()
    print("V3 experimental Simulation ended")


if __name__ == "__main__":
    main()
