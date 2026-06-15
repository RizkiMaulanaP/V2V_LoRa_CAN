import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import ExtendedKalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints
from math import sin, cos, radians, degrees

def lat_lon_to_enu(lat, lon, origin_lat, origin_lon):
    R = 6371000 
    phi_0 = radians(origin_lat)
    return (radians(lon - origin_lon) * R * cos(phi_0), radians(lat - origin_lat) * R)

# --- Shared Transition and Measurement Models ---
def fx(x, dt, gz=0, ax=0):
    """ State transition function used by UKF """
    e, n, v, psi = x[0], x[1], x[2], x[3]
    return np.array([
        e + v * cos(psi) * dt + 0.5 * ax * cos(psi) * dt**2,
        n + v * sin(psi) * dt + 0.5 * ax * sin(psi) * dt**2,
        v + ax * dt,
        psi + gz * dt
    ])

def h_gps(x): return np.array([x[0], x[1]])
def h_obd(x): return np.array([x[2]])

# --- EKF Class with Jacobian ---
class VehicleEKF(ExtendedKalmanFilter):
    def __init__(self):
        super().__init__(dim_x=4, dim_z=2)
        self.Q = np.diag([0.01, 0.01, 0.1, 0.01])

    def get_F(self, x, dt):
        v, psi = x[2], x[3]
        F = np.eye(4)
        F[0, 2] = cos(psi) * dt
        F[0, 3] = -v * sin(psi) * dt
        F[1, 2] = sin(psi) * dt
        F[1, 3] = v * cos(psi) * dt
        return F

# --- Main Fusion Engine ---
def run_comparison(csv_path):
    df = pd.read_csv(csv_path).sort_values('mcu_ts_ms')
    df['t'] = df['mcu_ts_ms'] / 1000.0
    
    # Initialize origin
    gps_init = df[df['frame_type'] == 'GPS'].iloc[0]
    origin = (gps_init['gps_lat'], gps_init['gps_lon'])

    # 1. Setup UKF
    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=1)
    ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=0.01, fx=fx, hx=h_gps, points=sigmas)
    ukf.x = np.array([0, 0, 0, 0])
    ukf.P = np.diag([1.0, 1.0, 1.0, 0.1])
    ukf.Q = np.diag([0.01, 0.01, 0.1, 0.01])
    
    # 2. Setup EKF
    ekf = VehicleEKF()
    ekf.x = np.array([[0, 0, 0, 0]]).T

    results = []
    last_t = df['t'].iloc[0]

    for _, row in df.iterrows():
        dt = row['t'] - last_t
        if dt <= 0 or dt > 1.0:
            last_t = row['t']
            continue
        
        # Get IMU inputs for prediction
        gz = row['gx_rads'] if row['frame_type'] == 'IMU' else 0 # Fixed column name from your sample
        ax = row['ax_ms2'] if row['frame_type'] == 'IMU' else 0


	# --- PREDICT ---
        # 1. Update UKF (The UKF class handles x and P internally)
        ukf.predict(dt=dt, gz=gz, ax=ax)

        # 2. Update EKF Manually
        # A. Update the state x using our non-linear motion model
        ekf.x = fx(ekf.x.flatten(), dt, gz, ax).reshape(-1, 1)
        
        # B. Calculate and assign the Jacobian F for this time step
        ekf.F = ekf.get_F(ekf.x.flatten(), dt)
        
        # C. Call predict to update the covariance P (using ekf.F and ekf.Q)
        ekf.predict()


        # --- UPDATE ---
        if row['frame_type'] == 'GPS' and row['fix_valid'] == 1:
            e, n = lat_lon_to_enu(row['gps_lat'], row['gps_lon'], *origin)
            z = np.array([e, n])
            ukf.update(z, R=np.diag([1.5, 1.5]))
            ekf.update(z.reshape(-1,1), HJacobian=lambda x: np.array([[1,0,0,0],[0,1,0,0]]),
                       Hx=lambda x: np.array([[x[0,0]], [x[1,0]]]), R=np.diag([1.5, 1.5]))
            
        elif row['frame_type'] == 'OBD':
            v_ms = row['obd_speed_kmh'] / 3.6
            ukf.update(np.array([v_ms]), hx=h_obd, R=np.array([[0.2]]))
            ekf.update(np.array([[v_ms]]), HJacobian=lambda x: np.array([[0,0,1,0]]),
                       Hx=lambda x: np.array([[x[2,0]]]), R=np.array([[0.2]]))

        results.append({
            't': row['t'],
            'ukf_e': ukf.x[0], 'ukf_n': ukf.x[1], 'ukf_v': ukf.x[2] * 3.6,
            'ekf_e': ekf.x[0,0], 'ekf_n': ekf.x[1,0], 'ekf_v': ekf.x[2,0] * 3.6,
            'obd_v': row['obd_speed_kmh'] if row['frame_type'] == 'OBD' else None
        })
        last_t = row['t']

    return pd.DataFrame(results)

# --- Visualization ---
def plot_results(res):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # 1. Cartesian Plane Path
    ax1.plot(res['ukf_e'], res['ukf_n'], label='UKF Path', color='blue', alpha=0.8)
    ax1.plot(res['ekf_e'], res['ekf_n'], label='EKF Path', color='red', linestyle='--')
    ax1.set_title("Vehicle Trajectory Comparison")
    ax1.set_xlabel("East (m)"); ax1.set_ylabel("North (m)")
    ax1.legend(); ax1.grid(True)
    
    # 2. Speed Comparison
    ax2.plot(res['t'], res['ukf_v'], label='Fused Speed (UKF)', color='blue')
    ax2.plot(res['t'], res['ekf_v'], label='Fused Speed (EKF)', color='red', linestyle='--')
    ax2.scatter(res['t'], res['obd_v'], label='Raw OBD (2Hz)', color='black', s=5, alpha=0.4)
    ax2.set_title("Speed Fusion Performance")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Speed (km/h)")
    ax2.legend(); ax2.grid(True)
    
    plt.show()

# Run:

res = run_comparison('raw_20260423_124435.csv'); plot_results(res)
