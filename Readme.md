**AAE6102 Assignment 1 Report**

 

### Task 1 – Acquisition

![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image002.png)First we perform acquisition of GPS signals from raw front-end data. Our code searches for signals from satellites specified in the settings structure by correlating the incoming signal with locally generated C/A codes. The function first processes the input signal (potentially resampling it to speed up acquisition), then performs a two-step acquisition process: a coarse search across frequency bins and code phases, followed by a fine resolution frequency search for detected signals. For each satellite, it calculates the correlation peak ratio to determine if a signal is present, and if found, it stores the code phase and carrier frequency in the output structure. 

 

 

###  

###  

 

 

 

 

 

 

 

 

 

 

![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image004.jpg)

![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image006.jpg)Fig.1 Acquisition results (Opensky)

### ![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image008.jpg)

Fig.2 Acquisition results (Urban)

###  

###  

###  

### Task 2 – Tracking

Adapt the **tracking loop (DLL)** to generate **correlation plots** and analyze the tracking performance. Discuss the impact of urban interference on the correlation peaks. *(Multiple correlators must be implemented for plotting the correlation function.)*

 

Then we perform tracking of GPS signals for channels identified by the acquisition process. We implement both code tracking (DLL - Delay Lock Loop) and carrier tracking (PLL - Phase Lock Loop) to follow the signal as it changes over time. For each channel with an assigned PRN, the function generates local replicas of early, prompt, and late versions of the C/A code, mixes the incoming signal to baseband using carrier wave estimates, and calculates correlation values. These correlation values feed into discriminators that determine tracking errors, which are then filtered and used to adjust the code and carrier frequency estimates in a feedback loop. The function collects tracking metrics throughout processing, including signal strength (C/No), correlator outputs, and loop discriminator values, updating a progress bar every 50ms. The output structure contains detailed tracking results for each millisecond of processing across all active channels.

![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image010.jpg)

 

Fig.3 Channel 1 (RPN 16) results (Opeksky)

 

 

![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image012.jpg)

 

 

 

 

 

 

 

 

 

 

 

 

Fig.![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image014.jpg)4 Channel 1 (RPN 16) CNo (Opensky)

###  

Fig.5 Channel 2 (RPN 26) result (Opensky)

### ![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image016.jpg)

Fig.6 Channel 2 (RPN 26) CNo (Opensky)

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image018.jpg)

Fig.7 Channel 1 (RPN 1) results (Urban)

![A screen shot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image020.jpg)

Fig.8 Channel 1 (RPN 1) CNo (Urban)

 

Urban multipath distorts these correlation functions:

Peak Asymmetry: The correlation triangle becomes asymmetric due to reflected signals, creating a bias in the detected peak position.

Peak Widening: The correlation function widens, reducing the sharpness of the peak and degrading range measurement precision.

Multiple Peaks: In severe multipath conditions, secondary peaks can appear, potentially causing the DLL to track an incorrect peak.

 

 

 

 

 

### Task 3 – Navigation Data Decoding

Decode the **navigation message** and extract key parameters, such as **ephemeris data**, for at least one satellite.

 

Decoding NAV for Urban dataset’s RPN 01:

TOW of first sub-frame: 449352 seconds
   Ephemeris parameters for PRN 01:
      C_ic: -7.4506e-08
    omega_0: -3.1060
      C_is: 1.6019e-07
      i_0: 0.9761
      C_rc: 287.4688
     omega: 0.7115
    omegaDot: -8.1696e-09
    IODE_sf3: 72
      iDot: -1.8108e-10
    idValid: [2 0 3]
   weekNumber: 1032
    accuracy: 0
     health: 0
      T_GD: 5.5879e-09
      IODC: 12
      t_oc: 453600
      a_f2: 0
      a_f1: -9.4360e-12
      a_f0: -3.4898e-05
    IODE_sf2: 72
      C_rs: -120.7188
     deltan: 4.1909e-09
      M_0: 0.5179
      C_uc: -6.3349e-06
       e: 0.0089
      C_us: 5.3011e-06
     sqrtA: 5.1537e+03
      t_oe: 453600
      TOW: 449352

 

The navigation part of our code takes tracking results for GPS satellites and calculates navigation solutions for the receiver. It first decodes ephemeris data from navigation messages in the tracking results, keeping only channels with complete ephemeris information. The function then processes measurement points sequentially, calculating pseudoranges between satellites and receiver at each point by comparing transmit and receive times. It determines satellite positions and clock corrections using the ephemeris data, then uses least squares estimation to solve for the receiver's position and clock error based on pseudorange measurements. For each valid position fix, the function converts the ECEF (Earth-Centered, Earth-Fixed) coordinates to geodetic (latitude, longitude, height) and UTM (Universal Transverse Mercator) coordinates, and calculates DOP (Dilution of Precision) values to indicate solution quality. The algorithm also applies satellite elevation masking, excluding satellites below a specified elevation angle from position calculations.

### Task 4 – Position and Velocity Estimation

Using **pseudorange measurements** from tracking, implement the **Weighted Least Squares (WLS)** algorithm to compute the **user's position and velocity**.

·  ![img](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image022.jpg)Plot the user **position** and **velocity**. 

 

Fig.9 Navigation results (Opensky)

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image024.jpg)

 

Fig.10 Navigation results (Urban)

 

·  Compare the results with the **ground truth**. 

 

Opensky:

| Label            | Latitude           | Longitude         |
| ---------------- | ------------------ | ----------------- |
| Ground Truth     | 22.328444770087565 | 114.1713630049711 |
| Calculated (WLS) | 22.19423931        | 114.10169524      |

 

The difference is 16.556 km.

 

 

Urban:

| Label            | Latitude    | Longitude        |
| ---------------- | ----------- | ---------------- |
| Ground Truth     | 22.3198722  | 114.209101777778 |
| Calculated (WLS) | 22.19104142 | 114.12273916     |

 

The difference is 16.86 km.

 

 

·  Discuss the impact of **multipath effects** on the WLS solution. 

 

We can see that errors are quite large. Multipath occurs when satellite signals reach the receiver via multiple paths due to reflections from buildings, terrain, or other surfaces. These reflected signals cause errors in pseudorange measurements. Effects on the WLS Solution include: Position Accuracy Degradation, Weighting Issues, Correlated Errors and Solution Instability.

 

 

**Task 5 – Kalman Filter-Based Positioning**

Develop an **Extended Kalman Filter (EKF)** using **pseudorange and Doppler measurements** to estimate **user position and velocity**.

 

We have written the applyEKF function that implements an Extended Kalman Filter (EKF) for GPS navigation, providing improved position estimates compared to the standard Weighted Least Squares (WLS) approach.

**Key Components of the function:**

1. **State     Vector**:
   - 4 states:      X, Y, Z position coordinates in ECEF frame, and receiver clock bias (dt)
   - Uses      a static position model with a random walk clock bias
2. **Initialization**:
   - Sets      initial state from the first valid WLS solution
   - Initializes      covariance matrices for state, process noise, and measurement noise
3. **Epoch     Processing**:
   - Special      handling for first 3 epochs during "initialization phase"
   - For      each epoch, it processes visible satellites to calculate: 
     - Predicted       pseudoranges
     - Jacobian       matrix of measurement model
     - Innovation       vector (difference between measured and predicted pseudoranges)
4. **Robustness     Features**:
   - Elevation-dependent      measurement weighting
   - Innovation      threshold check (1000m, increased to 5000m during initialization)
   - Limiting      state corrections during initialization for stability
   - Higher      process noise during initialization to allow faster convergence
   - Handling      of singularities in matrix inversions
5. **Coordinate     Conversions**:
   - ECEF      to geodetic (latitude, longitude, height)
   - ECEF      to UTM (E, N, U)
6. **DOP Calculation**:
   - Computes      Dilution of Precision values (GDOP, PDOP, HDOP, VDOP, TDOP)

The function provides a more stable and accurate position solution than WLS by:

1. Filtering     out measurement noise
2. Using     the time history of measurements
3. Correctly     modelling the system dynamics
4. Handling     larger errors during initialization
5. Being     more robust to outliers in the measurements

 

 

 

 

 

 

 

Opensky:

| Label            | Latitude           | Longitude         |
| ---------------- | ------------------ | ----------------- |
| Ground Truth     | 22.328444770087565 | 114.1713630049711 |
| Calculated (EKF) | 22.3284            | 114.1717          |

 

The difference is 0.03502 km.

The position error was reduced by 99.79% by using the EKF approach.

 

 

 

Urban:

| Label            | Latitude   | Longitude        |
| ---------------- | ---------- | ---------------- |
| Ground Truth     | 22.3198722 | 114.209101777778 |
| Calculated (EKF) | 22.3196    | 114.208          |

 

The difference is 0.1173 km.

The position error was reduced by 99.30% by using the EKF approach

 

 

We can see that measurement error in the Opensky dataset is much smaller than Urban Dataset due to the absence of signal obstructions.

 

Output visualization for Urban dataset:

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image026.gif)

Fig. 11 Comparison of Latitude, Longitude and Height results derived by WLS (blue dots) method and EKF (red line) method (Urban)

![A screen shot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image028.gif)

Fig. 12 Comparison of North, East, Up results derived by WLS (blue dots) method and EKF (red line) method (Urban)

 

 

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image030.gif)

Fig. 13 Comparison of Position Error Standard Deviation from by WLS (blue dots) method and EKF (red line) method (Urban)

 

 

Output visualization for Opensky dataset:

 

![A screen shot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image032.gif)

Fig. 14 Comparison of Latitude, Longitude and Height results derived by WLS (blue dots) method and EKF (red line) method (Opensky)

 

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image034.gif)

Fig. 15 Comparison of North, East, Up results derived by WLS (blue dots) method and EKF (red line) method (Opensky)

 

![A screenshot of a computer  AI-generated content may be incorrect.](https://github.com/dongzhesu/AAE6102Homework1/blob/main/files/clip_image036.gif)

Fig. 16 Comparison of Position Error Standard Deviation from by WLS (blue dots) method and EKF (red line) method (Opensky)

 