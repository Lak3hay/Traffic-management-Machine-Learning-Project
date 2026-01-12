import traci
import csv
import statistics

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"
SUMOCFG = "config/grid.sumocfg"

CONTROL_INTERVAL = 10
MIN_GREEN = 15
MAX_GREEN = 60
MAX_SIM_TIME = 2000

ALPHA = 1.0

BETA_MIN, BETA_MAX = 0.3, 0.9
GAMMA_MIN, GAMMA_MAX = 0.1, 0.5

FAIRNESS_LIMIT = 60
# ---------------------------------------


def lane_queue(lane_id):
    return traci.lane.getLastStepHaltingNumber(lane_id)


def compute_adaptive_beta(q_up, q_down):
    raw = q_down / (q_up + 1.0)
    return min(BETA_MAX, max(BETA_MIN, raw))


def compute_adaptive_gamma(max_age):
    raw = max_age / FAIRNESS_LIMIT
    return min(GAMMA_MAX, max(GAMMA_MIN, raw))


def compute_phase_pressure(tls_id, phase_index, fairness_age,
                           state_writer, step):
    logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)[0]
    phase = logic.phases[phase_index]
    links = traci.trafficlight.getControlledLinks(tls_id)

    ups, downs, ages = [], [], []

    for link_group, signal in zip(links, phase.state):
        if signal.lower() != 'g':
            continue
        for in_lane, out_lane, _ in link_group:
            ups.append(lane_queue(in_lane))
            downs.append(lane_queue(out_lane) if out_lane else 0)
            ages.append(fairness_age.get(in_lane, 0))

    if not ups:
        return 0.0, 0.0, 0.0, 0.0

    q_up = sum(ups)
    q_down = sum(downs)
    max_age = max(ages)

    beta = compute_adaptive_beta(q_up, q_down)
    gamma = compute_adaptive_gamma(max_age)

    pressure = ALPHA * q_up - beta * q_down + gamma * max_age

    state_writer.writerow([
        step, tls_id, phase_index,
        q_up, q_down, max_age,
        beta, gamma, pressure
    ])

    return max(pressure, 0.0), beta, gamma, max_age


def main():
    traci.start([SUMO_BINARY, "-c", SUMOCFG, "--start"])
    print("V5 Adaptive β(t)/γ(t) Max-Pressure started")

    tls_ids = traci.trafficlight.getIDList()
    last_switch = {tls: 0 for tls in tls_ids}
    fairness_age = {tls: {} for tls in tls_ids}

    perf_log = open("results_v5_experiment.csv", "w", newline="")
    perf_writer = csv.writer(perf_log)
    perf_writer.writerow(["time", "avg_speed", "running", "halted"])

    state_log = open("state_log_v5_experiment.csv", "w", newline="")
    state_writer = csv.writer(state_log)
    state_writer.writerow([
        "time", "tls", "phase",
        "q_up", "q_down", "max_age",
        "beta", "gamma", "pressure"
    ])

    ctrl_log = open("control_log_v5_experiment.csv", "w", newline="")
    ctrl_writer = csv.writer(ctrl_log)
    ctrl_writer.writerow([
        "time", "tls", "selected_phase",
        "pressure_best", "pressure_second",
        "pressure_gap",
        "max_age", "avg_age",
        "beta", "gamma",
        "phase_switched"
    ])

    switch_log = open("switch_reason_v5_experiment.csv", "w", newline="")
    switch_writer = csv.writer(switch_log)
    switch_writer.writerow([
        "time", "tls",
        "prev_phase", "new_phase",
        "switch_reason"
    ])

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

            pressures, betas, gammas, ages = [], [], [], []

            for p in range(num_phases):
                pr, b, g, a = compute_phase_pressure(
                    tls, p, fairness_age[tls],
                    state_writer, step
                )
                pressures.append(pr)
                betas.append(b)
                gammas.append(g)
                ages.append(a)

            best_phase = pressures.index(max(pressures))
            sorted_p = sorted(pressures, reverse=True)

            current_phase = traci.trafficlight.getPhase(tls)

            pressure_wants_switch = (best_phase != current_phase)
            tmax_forces_switch = (elapsed >= MAX_GREEN)

            if pressure_wants_switch or tmax_forces_switch:
                if tmax_forces_switch and not pressure_wants_switch:
                    reason = "TMAX"
                else:
                    reason = "PRESSURE"

                traci.trafficlight.setPhase(tls, best_phase)
                last_switch[tls] = step

                switch_writer.writerow([
                    step, tls,
                    current_phase, best_phase,
                    reason
                ])

                for link_group in traci.trafficlight.getControlledLinks(tls):
                    for in_lane, _, _ in link_group:
                        fairness_age[tls][in_lane] = 0

            for link_group in traci.trafficlight.getControlledLinks(tls):
                for in_lane, _, _ in link_group:
                    fairness_age[tls].setdefault(in_lane, 0)
                    fairness_age[tls][in_lane] += 1

            ctrl_writer.writerow([
                step,
                tls,
                best_phase,
                sorted_p[0],
                sorted_p[1] if len(sorted_p) > 1 else 0,
                sorted_p[0] - (sorted_p[1] if len(sorted_p) > 1 else 0),
                max(ages),
                statistics.mean(ages),
                betas[best_phase],
                gammas[best_phase],
                int(pressure_wants_switch)
            ])

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
    print("V5 Simulation ended")


if __name__ == "__main__":
    main()
