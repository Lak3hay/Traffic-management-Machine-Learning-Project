import traci
import csv
import statistics

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"
SUMOCFG = "config/grid.sumocfg"

CONTROL_INTERVAL = 10
MIN_GREEN = 15
MAX_GREEN = 40
MAX_SIM_TIME = 2000

ALPHA = 1
BETA = 0.7
GAMMA = 0.3
# ---------------------------------------

STATE_LOG_FILE = "state_log_v4_experiment.csv"


def lane_queue(lane_id):
    return traci.lane.getLastStepHaltingNumber(lane_id)


def main():
    traci.start([SUMO_BINARY, "-c", SUMOCFG, "--start"])
    print("V4 Fairness + Downstream Aware Max-Pressure started")

    tls_ids = traci.trafficlight.getIDList()
    last_switch = {tls: 0 for tls in tls_ids}
    fairness_age = {tls: {} for tls in tls_ids}

    # ---------- PERFORMANCE CSV ----------
    perf_log = open("results_v4_experiment.csv", "w", newline="")
    perf_writer = csv.writer(perf_log)
    perf_writer.writerow(["time", "avg_speed", "running", "halted"])

    # ---------- STATE CSV ----------
    state_log = open(STATE_LOG_FILE, "w", newline="")
    state_writer = csv.writer(state_log)
    state_writer.writerow(["time", "tls", "phase", "qi", "qj", "ai"])

    # ---------- CONTROL CSV ----------
    ctrl_log = open("control_log_v4_experiment.csv", "w", newline="")
    ctrl_writer = csv.writer(ctrl_log)
    ctrl_writer.writerow([
        "time", "tls",
        "selected_phase",
        "pressure_best", "pressure_second",
        "pressure_gap",
        "max_age", "avg_age",
        "phase_switched"
    ])

    # ---------- SWITCH-REASON CSV ----------
    switch_log = open("switch_reason_v4_experiment.csv", "w", newline="")
    switch_writer = csv.writer(switch_log)
    switch_writer.writerow([
        "time", "tls",
        "prev_phase", "new_phase",
        "switch_reason"
    ])
    # --------------------------------------

    step = 0

    while step < MAX_SIM_TIME:
        traci.simulationStep()
        step += 1

        for tls in tls_ids:
            elapsed = step - last_switch[tls]

            if elapsed < CONTROL_INTERVAL or elapsed < MIN_GREEN:
                continue

            logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls)[0]
            links = traci.trafficlight.getControlledLinks(tls)
            num_phases = len(logic.phases)

            pressures = []
            ages = []

            # -------- PHASE EVALUATION --------
            for p in range(num_phases):
                qi_sum = qj_sum = ai_sum = 0

                for link_group, signal in zip(links, logic.phases[p].state):
                    if signal.lower() != 'g':
                        continue

                    for in_lane, out_lane, _ in link_group:
                        qi_sum += lane_queue(in_lane)
                        qj_sum += lane_queue(out_lane) if out_lane else 0
                        ai_sum += fairness_age[tls].get(in_lane, 0)

                state_writer.writerow([
                    step, tls, p, qi_sum, qj_sum, ai_sum
                ])

                pr = max(ALPHA * qi_sum - BETA * qj_sum + GAMMA * ai_sum, 0.0)
                pressures.append(pr)
                ages.append(ai_sum)
            # ---------------------------------

            best_phase = pressures.index(max(pressures))
            sorted_p = sorted(pressures, reverse=True)

            current_phase = traci.trafficlight.getPhase(tls)
            switched = (best_phase != current_phase)

            # -------- SWITCH WITH CORRECT REASON --------
            if switched and elapsed < MAX_GREEN:
                reason = "PRESSURE"

                traci.trafficlight.setPhase(tls, best_phase)
                last_switch[tls] = step

                switch_writer.writerow([
                    step, tls, current_phase, best_phase, reason
                ])

                for link_group in links:
                    for in_lane, _, _ in link_group:
                        fairness_age[tls][in_lane] = 0

            elif elapsed >= MAX_GREEN:
                reason = "TMAX"

                traci.trafficlight.setPhase(tls, best_phase)
                last_switch[tls] = step

                switch_writer.writerow([
                    step, tls, current_phase, best_phase, reason
                ])

                for link_group in links:
                    for in_lane, _, _ in link_group:
                        fairness_age[tls][in_lane] = 0
            # --------------------------------------------

            # increment fairness age
            for link_group in links:
                for in_lane, _, _ in link_group:
                    fairness_age[tls].setdefault(in_lane, 0)
                    fairness_age[tls][in_lane] += 1

            # -------- CONTROL LOG --------
            ctrl_writer.writerow([
                step,
                tls,
                best_phase,
                sorted_p[0],
                sorted_p[1] if len(sorted_p) > 1 else 0,
                sorted_p[0] - (sorted_p[1] if len(sorted_p) > 1 else 0),
                max(ages),
                statistics.mean(ages),
                int(switched)
            ])
            # ----------------------------

        # ---------- PERFORMANCE ----------
        veh_ids = traci.vehicle.getIDList()
        running = len(veh_ids)

        if running > 0:
            avg_speed = sum(traci.vehicle.getSpeed(v) for v in veh_ids) / running
            halted = sum(1 for v in veh_ids if traci.vehicle.getSpeed(v) < 0.1)
        else:
            avg_speed = 0.0
            halted = 0

        perf_writer.writerow([step, avg_speed, running, halted])

        if step % 20 == 0:
            print(f"t={step:4d}s | avg_speed={avg_speed:5.2f} | running={running}")

        if traci.simulation.getMinExpectedNumber() == 0:
            break

    perf_log.close()
    state_log.close()
    ctrl_log.close()
    switch_log.close()
    traci.close()
    print("V4 Simulation ended")


if __name__ == "__main__":
    main()
