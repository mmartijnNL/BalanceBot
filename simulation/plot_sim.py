import matplotlib.pyplot as plt
import pandas as pd



# Load the simulation results (robust to being run from project root or simulation)
import os
csv_path = "sim_stabilize.csv"
if not os.path.exists(csv_path):
    csv_path = os.path.join(os.path.dirname(__file__), "sim_stabilize.csv")
df = pd.read_csv(csv_path)

plt.figure(figsize=(10, 6))
plt.plot(df['t'], df['pitch_deg'], label='Pitch (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.title('BalanceBot Simulation: Pitch Angle vs. Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
