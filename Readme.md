# Quadcopter Altitude Control using MATLAB 🚁

This project showcases altitude control of a quadcopter using:
- PID Controllers for altitude, attitude, and position
- Extended Kalman Filter (EKF) for state estimation
- MATLAB simulations, state-space modeling, and visualizations

> Developed by **Ayush Jain**, Birla Institute of Technology and Science, Pilani

---

## 📁 Project Structure

| Folder/File                     | Description |
|--------------------------------|-------------|
| `src/`                         | MATLAB scripts for simulation and control logic |
| `data/`                        | Pre-saved `.mat` files for state-space models |
| `Quadcopter_PID_EKF.mlx`       | MATLAB Live Script combining simulations and plots |
| `README.md`                    | This project summary |

---

## 🧠 Key Components

### 🔹 1. PID Controllers
- **`pid_altitude_control.m`**: Altitude stabilization
- **`pid_attitude_control.m`**: Roll/pitch/yaw orientation control
- **`pid_position_control.m`**: Position hold and path tracking

### 🔹 2. EKF Estimation
- **`ekf_state_estimation.m`**: Implements Extended Kalman Filter for noisy sensor fusion and real-time state estimation.

### 🔹 3. State-Space Modeling
- **`quadcopter_state_space.m`**: Defines A, B, C, D matrices and simulates system dynamics.
- **`quadcopter_state_space.mat`** & **`quadcopter_sys.mat`**: Precomputed data used in simulations.

### 🔹 4. Open Loop Simulation
- **`simulate_open_loop.m`**: Visualizes system response without control feedback.

---

## 💻 Tools Used

- MATLAB R2024a
- Control System Toolbox
- Signal Processing Toolbox
- Live Editor (`.mlx` scripting)
- Simulink (optional, if added later)

---

## 🧪 Getting Started

1. Clone the repo:
   ```bash
   git clone https://github.com/YOUR_USERNAME/quadcopter-altitude-control.git
   cd quadcopter-altitude-control
