# ✈️ 6-DOF Flight Dynamics Simulator 
A beginner-friendly, interview-ready C++ project that simulates a tiny aircraft's motion in 3D using essential flight-dynamics equations and renders an artificial horizon with Qt. 

- Mirrors real simulator architecture (physics ↔ display separation).

## 🧠 Core physics (kept simple)

### Dynamic pressure
```
q = ½ρV², V = √(u² + v² + w²), α = atan2(w,u)
```

### Lift & Drag
```
L = C_L·q·S,  C_L = C_L0 + C_Lα·α
D = C_D·q·S,  C_D = C_D0 + k·C_L²
```

### Forces (body axes; X forward, Y right, Z down)
```
X ≈ T - D, Y ≈ 0, Z ≈ -L + W_B
```
with weight rotated into body axes.

### Translational dynamics (body frame)
```
m·v̇_B = F_B - ω × (m·v_B)
```

### Position in NED
```
ṙ_I = C_IB·v_B
```

### Rotational dynamics (principal inertias)
```
I·ω̇ = τ - ω × (I·ω)
```

### Control moments (toy)
```
L = K_ℓ·δ_a,  M = K_m·δ_e,  N = K_n·δ_r
```

### Euler-angle kinematics
```
φ̇ = p + tan θ·(q sin φ + r cos φ)
θ̇ = q cos φ - r sin φ
ψ̇ = (q sin φ + r cos φ)/cos θ
```

**Integration**: start with Euler (simple), upgrade to RK4 later.

## 🧵 Concurrency design

- **Physics thread (producer)**: fixed step (e.g., 200 Hz), integrates state, pushes to SPSC (single-producer/single-consumer) ring buffer. Never blocks (drops if full).
- **Qt UI thread (consumer)**: ~60 Hz timer pulls latest state and renders an artificial horizon (uses roll & pitch).
- **Main**: starts/stops threads.



## 🍎 macOS setup & build  (TODO Portability)

### Install tools
```bash
brew install cmake qt
```

### Configure & build
```bash
mkdir -p build && cd build
cmake -DQt6_DIR=$(brew --prefix qt)/lib/cmake/Qt6 ..
cmake --build . -j
```

### Run
```bash
./flightSim
```


## 🔢 Default parameters (tweak freely)

- Mass m = 1200 kg, inertias Ix=1200, Iy=1500, Iz=1800 kg·m²
- Wing S=16 m², CL0=0.2, CL_a=5.5 rad⁻¹, CD0=0.03, k=0.07
- Moments: Kℓ=3000, Km=5000, Kn=2000
- Air rho=1.225 kg/m³, gravity g=9.81 m/s²
- Controls in [-1,1]

