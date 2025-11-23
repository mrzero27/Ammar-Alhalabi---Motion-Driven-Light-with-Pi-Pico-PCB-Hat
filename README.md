
# Motion-Driven Water Line – Raspberry Pi Pico (Dual Core)

## 1. What the project does

This project runs on a **Raspberry Pi Pico (RP2040)** and drives a **WS2812B (NeoPixel) LED strip** to display a moving “water line” that reacts to the board’s **tilt**.

- If an **MPU-6050** IMU is connected:
  - The code reads accelerometer + gyroscope data over I²C.
  - It computes the **roll angle** (left/right tilt) and uses it to move a “water blob” along the LED strip.
- If **no sensor is present** or it cannot be detected:
  - The code automatically switches to a **simulated motion mode** that generates a smooth sine-wave roll angle.
  - The program still runs and shows the water animation and ASCII output, even on a bare Pico with only USB.

In all cases:

- A **NeoPixel strip** shows a blue-green “water blob” that sits around the middle and moves left/right with tilt, with soft edges and a small shimmer effect.
- An **ASCII visualization** of the water line is printed to the **Serial Monitor** once per frame, together with live sensor/simulation values.

---

## 2. How to test it (with or without hardware)

### 2.1 With full hardware (Pico + MPU-6050 + WS2812B)

**Connections (default, matching the code):**

**MPU-6050 → Pico (I²C)**

- `VCC` → `3V3(OUT)`
- `GND` → `GND`
- `SDA` → `GP4`
- `SCL` → `GP5`
- `AD0` → `GND` (so the address is `0x68` as used in the code)

**WS2812B LED strip → Pico**

- `DIN` → `GP2`
- `GND` → `GND`
- `VCC` → 5 V or 3.3 V (according to your strip’s requirements; GND must be common with Pico)

**Test procedure:**

1. Connect the Pico to the PC via USB.
2. Open the Arduino sketch and upload it using **Arduino IDE** (board set to **Raspberry Pi Pico**, baud rate 115200).
3. Open **Serial Monitor** at **115200 baud**.
4. On startup, you should see something like:

   ```text
   MPU-6050 detected, using real sensor.

5. Now gently tilt the board left and right:

   * The **roll angle** in the Serial Monitor should change sign and magnitude.
   * The **ASCII water bar** in Serial should move left/right accordingly.
   * The **water blob on the NeoPixel strip** should visually follow the tilt.

This demonstrates the full behavior with real sensor data.

---

### 2.2 Without IMU / without custom PCB (simulation mode)

You can still run and test the project **without any external hardware**:

1. Connect only the **Pico + USB** to your PC (no MPU, no LEDs required).
2. Upload the sketch.
3. Open the **Serial Monitor** at **115200 baud**.

If the IMU is not found, you will see:

```text
MPU-6050 NOT found, falling back to simulated motion.
```

After that:

* The code generates a **simulated roll angle** using a sine wave (no sensor needed).
* Every frame it prints:

  * The simulated roll angle.
  * Fixed accelerometer values (`ax, ay, az`).
  * `src=SIM` to indicate simulation mode.
  * The ASCII water bar, which moves smoothly left/right over time.

If you connect only a **NeoPixel strip** (and no IMU), you will also see the LED water animation driven by the same simulated motion.

This way the project is fully testable both:

* **With real hardware** (IMU + LEDs).
* **With no hardware at all** (bare Pico + Serial Monitor).

---

## 3. How each core is used

The Raspberry Pi Pico has two cores. This project explicitly splits responsibilities:

### Core 0 – Sensor / Simulation and Shared State

Core 0 runs the standard Arduino `setup()` and `loop()` functions.

**Responsibilities:**

* Initialize:

  * Serial at 115200 baud.
  * I²C (using `Wire.setSDA(4)` and `Wire.setSCL(5)`).
  * The MPU-6050 (if present) by pinging its I²C address and waking it from sleep.
  * The NeoPixel strip (pin `GP2`, `LED_COUNT = 24`).
  * Launch **Core 1** using `multicore_launch_core1(core1Task);`.

* In `loop()` (every `SENSOR_INTERVAL_MS = 10 ms`):

  1. Create a local `MotionSample` struct.
  2. If the IMU is present and `readMPU6050(...)` succeeds:

     * Read raw accel/gyro registers.
     * Convert raw values to physical units:

       * Accel in g (`raw / 16384.0`).
       * Gyro in deg/s (`raw / 131.0`).
     * Compute the **roll angle**:
       `rollDeg = atan2(ay, az) * 180 / PI;`
     * Mark `sensorOk = true`.
  3. Otherwise:

     * Call `generateSimulatedMotion(...)` to compute a **sine-wave roll angle** (±30°) with fixed accel values.
     * Mark `sensorOk = false` (simulation mode).
  4. Store the resulting `MotionSample` into the global `g_motion` struct, which is shared with Core 1.

**Summary:** Core 0 is responsible for **“what is the current motion?”**, using either real IMU data or a simulated pattern, and publishes that as a shared state.

---

### Core 1 – LED Rendering and ASCII Output

Core 1 runs the `core1Task()` function in parallel.

**Responsibilities (every `FRAME_INTERVAL_MS = 50 ms`):**

1. Copy the shared `g_motion` into a local variable inside a `noInterrupts()/interrupts()` block to avoid partial updates.
2. Take `rollDeg` from the motion sample and map it to a normalized tilt value in **[-1, 1]** using `mapRollToTilt()`:

   * Clamp roll to ±45°.
   * Divide by 45° to get −1…+1.
3. Call `updateLeds(tiltNorm)`:

   * Compute a **center index** along the LED strip based on the tilt.
   * Reflect the center when it would move off the ends (simple “bounce” effect).
   * Apply a **soft radius** around the center to create a blob (triangular falloff).
   * Add a small **shimmer** (time-based modulation) to make the water look alive.
   * Set NeoPixel colors to a blue-ish water tone and call `strip.show()`.
4. Call `buildAsciiWaterLine(tiltNorm)`:

   * Use the same conceptual center and radius over a string of length `LED_COUNT` (or 20 as fallback).
   * Write:

     * `O` at the center.
     * `~` and `-` around the center within the radius.
     * `_` elsewhere.
   * Returns a single-line ASCII representation of the water.
5. Print a single line to Serial combining:

   * Roll angle.
   * `ax, ay, az`.
   * The source (`MPU` or `SIM`).
   * The ASCII water bar.

**Summary:** Core 1 is responsible for **visualization**, both on the **LED strip** and in the **Serial Monitor**, using the motion state produced by Core 0.

---

## 4. Sample Serial Monitor output

### Example with a detected MPU-6050 (real sensor data)

```text
MPU-6050 detected, using real sensor.
roll=-7.5 deg | ax=0.01g ay=-0.13g az=0.98g | src=MPU | water: ____---~~O~~---_________
roll= 0.2 deg | ax=0.00g ay= 0.00g az=1.00g | src=MPU | water: ______---~~O~~---_______
roll=18.9 deg | ax=0.03g ay= 0.32g az=0.94g | src=MPU | water: _________---~~O~~---____
```

### Example in simulation mode (no IMU / not detected)

```text
MPU-6050 NOT found, falling back to simulated motion.
roll=  9.4 deg | ax=0.00g ay=0.00g az=1.00g | src=SIM | water: _______---~~O~~---______
roll= 28.1 deg | ax=0.00g ay=0.00g az=1.00g | src=SIM | water: __________---~~O~~--___
roll= -5.6 deg | ax=0.00g ay=0.00g az=1.00g | src=SIM | water: ____---~~O~~---_________
```

Field meanings:

* `roll`: roll angle in degrees (negative = tilt one way, positive = tilt the other).
* `ax, ay, az`: acceleration components in g.
* `src`:

  * `MPU` = real MPU-6050 readings.
  * `SIM` = internal simulated motion.
* `water`: ASCII representation of the LED strip:

  * `O` = center of the blob.
  * `~` and `-` = body and edges of the water.
  * `_` = empty space.

---

## 5. PCB Layout & 3D Views

### 5.1 Top 3D view – Pico plugged into the hat

![Top 3D view – Pico hat](https://github.com/mrzero27/Ammar-Alhalabi---Motion-Driven-Light-with-Pi-Pico-PCB-Hat/blob/main/3D%20View%20Top.png)

This render shows the **Raspberry Pi Pico** plugged into the custom hat.
You can see the **stacking headers** on both sides, the LEDs along the Pico edge, and the overall mechanical outline with four mounting holes.
The silkscreen at the bottom credits the designer and clearly labels the board.

---

### 5.2 Bottom 3D view – IMU and passives

![Bottom 3D view – IMU section](https://github.com/mrzero27/Ammar-Alhalabi---Motion-Driven-Light-with-Pi-Pico-PCB-Hat/blob/main/3D%20View%20Bottom.png)

This is the **bottom side** of the PCB.
In the middle you can see the **MPU-6050** IMU and its surrounding capacitors and resistors (decoupling and pull-ups).
The four long female headers give access to all Pico GPIO pins while keeping the IMU wiring short and symmetric.

---

### 5.3 Top copper/layout view – routing and keep-out

![2D layout – routing](https://github.com/mrzero27/Ammar-Alhalabi---Motion-Driven-Light-with-Pi-Pico-PCB-Hat/blob/main/PCB%20Layout.png)

This 2D layout view shows the **copper pours, traces and keep-out area** under the Pico’s antenna.
You can see how the I²C lines from the Pico are routed cleanly to the IMU, how the ground pour ties everything together, and how the four mounting holes define the mechanical footprint.

---

### 5.4 Schematic – Pico, IMU and indicators

![Schematic – Pico + MPU-6050](https://github.com/mrzero27/Ammar-Alhalabi---Motion-Driven-Light-with-Pi-Pico-PCB-Hat/blob/main/Schematic.png)

The schematic shows the overall design:

* On the left, the **MPU-6050** with its power decoupling capacitors, I²C pull-up resistors and address pin configuration.
* At the top, **SDA/SCL indicator LEDs** with series resistors so you can visually see I²C activity.
* On the right, the **Raspberry Pi Pico** with both 20-pin headers broken out and power rails (3V3, VSYS/VBUS, GND) distributed to the connectors.

These four images together document how the PCB implements the firmware concept in hardware: Pico in the center, IMU close and quiet, and all GPIOs still available through the side headers.

```
