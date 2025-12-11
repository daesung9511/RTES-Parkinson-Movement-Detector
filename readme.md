# Parkinson Movement Detection System  
**Real-Time FFT-Based Classification of Tremor, Dyskinesia, and Freezing of Gait  
Using the DISCO-L475VG-IOT01A Development Board**

---

## 1. Introduction

Parkinson’s Disease (PD) presents several clinically important motor symptoms, including:

- **Tremor (3–5 Hz):** Rhythmic shaking of the hand or wrist  
- **Dyskinesia (5–7+ Hz):** Excessive involuntary movement due to high dopamine medication  
- **Freezing of Gait (FOG):** Sudden inability to step despite intention  

This project implements a **fully on-board real-time movement classifier** using the internal **Accelerometer + Gyroscope (LSM6DSL)** on the DISCO-L475VG-IOT01A.

All processing is performed on the embedded system:

- Sensor sampling  
- Preprocessing  
- FFT (CMSIS DSP)  
- Frequency band extraction  
- Clinical-rule classification  
- LED-based condition output  

No external hardware is used.

---

## 2. Features

### ✓ Detects 3 major Parkinsonian symptoms:
- **Tremor** (3–5 Hz rotational oscillations)  
- **Dyskinesia** (5–7 Hz rotational bursts)  
- **Freezing of gait** (low gait amplitude + high jitter)

### ✓ Runs entirely on embedded hardware  
### ✓ FFT-based classification (256‑point real FFT)  
### ✓ Uses gyroscope for tremor/dyskinesia and accelerometer for gait  
### ✓ 3-second sliding analysis window  
### ✓ Onboard LED output  

---

## 3. Hardware Components

| Component | Role |
|----------|------|
| **DISCO-L475VG-IOT01A board** | MCU + IMU + LEDs |
| **LSM6DSL IMU** | 3-axis accelerometer + 3-axis gyroscope |
| **LED1, LED2, LED3** | Tremor, Dyskinesia, Freezing indicators |
| **USB / Power bank** | Portable power |

Sensor configuration used:

- Accelerometer: **±2g @ 52 Hz**  
- Gyroscope: **±250 dps @ 52 Hz**  

---

## 4. System Architecture

```
   Sensors (Accelerometer + Gyroscope)
                    │
                    ▼
             Sampling @ 52Hz
        accel_mag / gyro_mag computed
                    │
                    ▼
              DC removal
              Zero padding
                    │
                    ▼
            CMSIS Fast Real FFT
                    │
                    ▼
      Frequency-band integration:
      - Tremor (3–5Hz, gyro)
      - Dyskinesia (5–7Hz, gyro)
      - Walk (0.5–3Hz, accel)
      - Fog  (3–8Hz, accel)
                    │
                    ▼
              Clinical Logic
                    │
                    ▼
           LED1 / LED2 / LED3 Output
```

---

## 5. Why Gyroscope + Accelerometer?

This project uses a clinically-inspired separation of sensor roles:

### **Gyroscope → Tremor & Dyskinesia**
Both tremor and dyskinesia are rotational:

- Gyro signals produce clean FFT peaks  
- Gyro is unaffected by gravity  
- Gyro avoids false walking contamination  
- Gyro provides higher SNR in 3–7 Hz  

### **Accelerometer → Walking & Freezing**
Walking is predominantly linear acceleration:

- Step cadence appears clearly at 0.5–3 Hz  
- Freezing causes drop in gait amplitude and rise in jitter  
- Fog ratio is based on accelerometer spectral energy  

This design mirrors real clinical Parkinson monitoring devices.

---

## 6. Sampling Details

| Parameter | Value |
|----------|--------|
| Sampling rate | **52 Hz** |
| Window length | **3 seconds** |
| Samples per window | 156 |
| FFT size | **256** |
| FFT resolution | **0.203 Hz/bin** |

---

## 7. Preprocessing

### 1. Magnitude calculation
```
accel_mag = sqrt(ax² + ay² + az²)
gyro_mag  = sqrt(gx² + gy² + gz²)
```
Magnitude removes orientation dependence.

### 2. Mean (DC) removal
Centering data improves FFT performance.

### 3. Zero-padding
Pads from 156 to 256 samples for FFT processing.

---

## 8. FFT & Frequency Bands

CMSIS-DSP routines used:
```
arm_rfft_fast_f32()
arm_cmplx_mag_f32()
```

### Frequency bands:

| Condition | Sensor | Hz Range |
|-----------|---------|-----------|
| Tremor | Gyroscope | **3–5 Hz** |
| Dyskinesia | Gyroscope | **5–7 Hz** |
| Walking | Accelerometer | **0.5–3 Hz** |
| Freeze Jitter | Accelerometer | **3–8 Hz** |

### Fog Ratio
```
fog_ratio = fog_power / (walk_power + 0.0001)
```

High fog ratio combined with low walk → freeze detection.

---

## 9. Classification Logic

### Tremor
```
tremor > threshold
AND walk is low
AND tremor > dyskinesia * 1.2
→ LED1 ON
```

### Dyskinesia
```
dyskinesia > threshold
AND walk is low
AND dyskinesia > tremor * 1.2
→ LED2 ON
```

### Freezing of Gait (FOG)
```
walk < 5
AND fog_ratio > 3
AND NOT dyskinesia
→ LED3 ON
```

Clinical constraints:

- Freeze + Tremor is possible  
- Freeze + Dyskinesia is physiologically impossible  

---

## 10. LED Indicators

| LED | Meaning |
|-----|----------|
| LED1 | **Tremor** detected |
| LED2 | **Dyskinesia** detected |
| LED3 | **Freezing of Gait** detected |

Priority:

1. Freeze dominates  
2. Tremor allowed during freeze  
3. Dyskinesia blocked during freeze  
4. Otherwise strongest band wins  

---

## 11. Demonstration Instructions

### Tremor Demonstration
- Hold the board lightly  
- Shake at **3–4 Hz**  
- LED1 turns on  

### Dyskinesia Demonstration
- Faster rotational shake (~6 Hz)  
- LED2 turns on  

### Walking Demonstration
- Move board up/down at ~1–2 Hz  
- All LEDs off  

### Freezing Demonstration
- Hold board almost still  
- Add small jitter  
- LED3 turns on  
- LED1 may also light (tremor+freeze allowed)  

---

## 12. Limitations

- 3-second window → detection latency  
- Thresholds may require tuning per user  
- High movement amplitude may saturate sensors  
- Classifier detects **presence**, not **severity**  

---

## 13. Future Work

- BLE transmission of movement metrics  
- TinyML for adaptive classification  
- Personalized threshold learning  
- Gait quality scoring  
- Continuous symptom trend analysis  

---

## 14. References

### Technical
- ST LSM6DSL Datasheet  
- CMSIS-DSP Library Documentation  
- Mbed OS API Reference  

### Clinical
- Moore et al., “Ambulatory Monitoring of FOG”  
- Griffiths et al., “Gyroscope-Based Dyskinesia Detection”  
- Deuschl et al., “Clinical Neurophysiology of Tremor”  

These studies guided the frequency-band design and classification rules.

---

## 15. Author

**Dae-Sung Jin**  
NYU Tandon School of Engineering  
Embedded Systems – Parkinson Movement Detection Project

---

## 16. Repository Structure

```
README.md
platformio.ini
/src
    main.cpp
```

---

## 17. Acknowledgements

This project was developed as part of the NYU Embedded Systems curriculum.  
All signal processing and classification run fully on the DISCO-L475 board as required.

