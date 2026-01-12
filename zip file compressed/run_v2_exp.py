import traci
import csv
import statistics

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"     # change to "sumo" for no GUI
SUMOCFG = "config/grid.sumocfg"

MIN_GREEN = 15
MAX_GREEN = 60
MAX_SIM_TIME = 2000
# ---------------------------------------


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
    traci.start([SUMO_BINARY, "-c", SUMOCFG])
    print("V2 Simulation started")

    # ---------- PERFORMANCE CSV ----------
    logfile = open("results_v2_experiment.csv", "w", newline="")
    writer = csv.writer(logfile)
    writer.writerow(["time", "avg_speed", "running", "halted"])

    # ---------- CONTROL LOG CSV ----------
    ctrl_log = open("control_log_v2_experiment.csv", "w", newline="")
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

    tls_ids = traci.trafficlight.getIDList()
    last_switch_time = {tls: 0 for tls in tls_ids}

    step = 0

    while traci.simulation.getMinExpectedNumber() > 0 and step < MAX_SIM_TIME:
        traci.simulationStep()
        step += 1

        for tls in tls_ids:
            links = traci.trafficlight.getControlledLinks(tls)

            incoming_lanes = set()
            for link_group in links:
                for link in link_group:
                    incoming_lanes.add(link[0])

            if not incoming_lanes:
                continue

            lane_queues = {
                lane: traci.lane.getLastStepHaltingNumber(lane)
                for lane in incoming_lanes
            }

            elapsed = step - last_switch_time[tls]
            if elapsed < MIN_GREEN:
                continue

            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
            num_phases = len(logic.phases)
            current_phase = traci.trafficlight.getPhase(tls)

            switched = 0
            next_phase = current_phase

            if elapsed >= MAX_GREEN or max(lane_queues.values()) > 0:
                next_phase = (current_phase + 1) % num_phases
                traci.trafficlight.setPhase(tls, next_phase)
                last_switch_time[tls] = step
                switched = 1

            # ---- CONTROL LOGGING (DECISION INSTANT) ----
            queues = list(lane_queues.values())
            ctrl_writer.writerow([
                step,
                tls,
                next_phase,
                max(queues),
                statistics.mean(queues),
                switched
            ])
            # -------------------------------------------

        # ---- PERFORMANCE LOGGING ----
        avg_speed, running = log_metrics(writer, step)

        if step % 10 == 0:
            print(
                f"t={step:4d}s | avg_speed={avg_speed:.2f} m/s | running={running}"
            )

    print("All vehicles cleared. V2 experimental Simulation ended.")
    logfile.close()
    ctrl_log.close()
    traci.close()


if __name__ == "__main__":
    main()
