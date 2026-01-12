import traci
import csv

# ---------------- CONFIG ----------------
SUMO_BINARY = "sumo-gui"      # use "sumo" for no GUI
SUMO_CFG = "config/grid.sumocfg"

MAX_SIM_TIME = 4000        # hard stop (seconds)
LOW_SPEED_THRESHOLD = 0.5     # m/s
LOW_SPEED_DURATION = 60       # seconds
# ---------------------------------------


def log_metrics(writer, step):
    vehicle_ids = traci.vehicle.getIDList()
    running = len(vehicle_ids)

    if running > 0:
        avg_speed = sum(traci.vehicle.getSpeed(v) for v in vehicle_ids) / running
        halted = sum(1 for v in vehicle_ids if traci.vehicle.getSpeed(v) < 0.1)
    else:
        avg_speed = 0.0
        halted = 0

    writer.writerow([step, avg_speed, running, halted])

#
def main():
    sumo_cmd = [
        SUMO_BINARY,
        "-c", SUMO_CFG
    ]

    traci.start(sumo_cmd)

    # ---- CSV LOGGING SETUP ----
    logfile = open("results_base_experimental.csv", "w", newline="")
    writer = csv.writer(logfile)
    writer.writerow(["time", "avg_speed", "running", "halted"])

    low_speed_start = None
    step = 0

    print("V1 Simulation started")

    # ---- SIMULATION LOOP ----
    while step < MAX_SIM_TIME:
        traci.simulationStep()
        step += 1

        veh_ids = traci.vehicle.getIDList()
        running = len(veh_ids)

        if running > 0:
            avg_speed = sum(traci.vehicle.getSpeed(v) for v in veh_ids) / running
        else:
            avg_speed = 0.0

        print(
            f"t={step:4d}s | avg_speed={avg_speed:.2f} m/s | running={running}"
        )

        # ---- LOG METRICS (KEY ADDITION) ----
        log_metrics(writer, step)

        # ---- GRIDLOCK DETECTION ----
        if avg_speed < LOW_SPEED_THRESHOLD and running > 0:
            if low_speed_start is None:
                low_speed_start = step
            elif step - low_speed_start >= LOW_SPEED_DURATION:
                print("\n==============================")
                print("SYSTEM FAILURE: GRIDLOCK")
                print(f"Failure time: {step} seconds")
                print("==============================\n")
                break
        else:
            low_speed_start = None

        # ---- ALL VEHICLES CLEARED ----
        if running == 0:
            print("\nAll vehicles cleared. Simulation ended.")
            break

    # ---- CLEAN SHUTDOWN ----
    logfile.close()
    traci.close()
    print("Simulation basemodel exp. closed cleanly")


if __name__ == "__main__":
    main()
