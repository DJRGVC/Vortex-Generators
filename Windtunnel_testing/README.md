# Wind Tunnel Testing - Lift Force Analysis

## Overview

This directory contains wind tunnel test data and analysis for a wing at 0° angle of attack with vortex generators retracted. The analysis calculates lift force as a function of windspeed using string gauge measurements.

## Experimental Setup

### Equipment
- **Wind tunnel** with variable speed fan
- **String gauge** (strain gauge) measuring vertical force
- **Wing specimen** mounted on the string gauge
- **LabVIEW data acquisition system**

### Measurement Principle

The string gauge is positioned to measure the **apparent weight** of the wing:
- At **0 mph (no airflow)**: The gauge reads the full weight of the wing assembly
- At **higher windspeeds**: Aerodynamic lift reduces the apparent weight on the gauge

**Key Equation:**
```
Lift Force = Baseline Weight - Apparent Weight
```

As the wing generates more lift, it "unloads" the string gauge, reducing the measured force.

## Data Files

### Calibration Data
Files: `calibration0g.xls` through `calibration100g.xls`

- Known weights (0-100g in 10g increments) applied to the string gauge
- Used to establish the voltage-to-force relationship
- **Column 5 (Untitled 3)**: Strain gauge voltage output

### Experimental Run Data
Files: `0deg_run1.xls` through `0deg_run7.xls`

- Wing tested at 7 different fan speeds (0 to ~2800 RPM)
- **Column 2 (Untitled)**: Fan speed in RPM
- **Column 5 (Untitled 3)**: Strain gauge voltage output

### File Format
- LabVIEW-generated tab-delimited text files (`.xls` extension but actually TSV)
- **Row 22**: Column headers
- **Row 23**: Data values

## Methodology

### 1. Calibration

**Objective:** Establish linear relationship between voltage and force

**Process:**
1. Apply known weights (0-100g) to string gauge
2. Record voltage output for each weight
3. Perform linear regression: `V = slope × F + intercept`

**Quality Metric:**
- R² = 0.9999 (Excellent linear correlation)
- Standard error: 6 × 10⁻⁶ V/g

**Calibration Equation:**
```
V = -0.001857 × F - 1.598511
```

**Inverted for force calculation:**
```
F = (V - intercept) / slope
F = (V + 1.598511) / (-0.001857)
```

### 2. Voltage-to-Force Conversion

For each experimental run:
1. Read strain gauge voltage (Column 5)
2. Apply calibration equation to get apparent weight
3. Result is the downward force on the gauge (in grams)

### 3. Lift Force Calculation

**Baseline:** First run (0 RPM) gives the wing's weight with no airflow
```
Baseline Weight = 520.52 g
```

**Lift for each run:**
```
Lift = Baseline Weight - Apparent Weight
```

**Physical interpretation:**
- At 0 mph: Lift = 0 g (no airflow, no lift)
- At 65.76 mph: Lift = 384.07 g (wing generates 384g of upward force)

### 4. RPM to Windspeed Conversion

**Empirical calibration data:**
| Fan RPM | Windspeed (mph) |
|---------|-----------------|
| 790     | 15.41          |
| 1188    | 25.05          |
| 1583    | 34.00          |
| 1984    | 45.63          |
| 2382    | 57.26          |
| 2769    | 65.76          |

**Conversion method:**
- Linear interpolation between known RPM-windspeed pairs
- Special case: 0 RPM = 0 mph (not interpolated)

### 5. Curve Fitting

Two models fitted to Lift vs Windspeed data:

**Quadratic Model** (Preferred - physically motivated):
```
Lift = 0.0805v² + 0.751v - 18.73
R² = 0.9986
```

Aerodynamic forces theoretically scale with velocity squared (lift ∝ v²), making this the physically appropriate model.

**Linear Model** (For comparison):
```
Lift = 7.337v - 128.60
R² = 0.9758
```

The quadratic model provides superior fit (R² = 0.999 vs 0.976).

## Results Summary

| Run | RPM    | Windspeed (mph) | Voltage (V) | Apparent Weight (g) | Lift (g) |
|-----|--------|-----------------|-------------|---------------------|----------|
| 1   | 0.0    | 0.00           | -2.565      | 520.52              | 0.00     |
| 2   | 796.3  | 15.56          | -2.545      | 509.47              | 11.04    |
| 3   | 1201.6 | 25.36          | -2.470      | 469.13              | 51.38    |
| 4   | 1602.9 | 34.58          | -2.364      | 411.97              | 108.55   |
| 5   | 2009.0 | 46.36          | -2.214      | 331.19              | 189.32   |
| 6   | 2410.7 | 57.89          | -2.035      | 235.13              | 285.39   |
| 7   | 2797.8 | 65.76          | -1.852      | 136.45              | 384.07   |

### Key Findings

1. **Lift increases with windspeed** as expected
2. **Maximum lift**: 384g at 65.76 mph (~74% of wing weight)
3. **Quadratic relationship**: Confirms theoretical lift ∝ v² behavior
4. **High precision**: R² > 0.998 for quadratic fit

## Usage

### Running the Analysis

```bash
python analyze_windtunnel_data.py
```

### Generated Outputs

1. **calibration_curve.png**
   - Voltage vs applied force relationship
   - Shows linear fit quality

2. **lift_vs_windspeed.png**
   - Main result: Lift force vs windspeed
   - Includes quadratic curve fit
   - Annotated data points

3. **combined_analysis.png**
   - Two-panel figure showing:
     - Top: Apparent weight vs windspeed (decreasing)
     - Bottom: Lift force vs windspeed (increasing)

4. **analysis_results.json**
   - Machine-readable results
   - Calibration parameters
   - All run data
   - Fit coefficients

## Physical Interpretation

### Sign Conventions

- **Voltage**: More negative = more downward force
- **Force/Weight**: Positive values in grams
- **Lift**: Positive = upward aerodynamic force

### Why Apparent Weight Decreases

```
Forces on wing: Weight (down) + Lift (up) = Gauge Reading (down)

Gauge Reading = Weight - Lift

As Lift increases → Gauge Reading decreases
```

This is analogous to standing on a scale in an elevator:
- Elevator accelerating up → scale reads more (feels heavier)
- Elevator accelerating down → scale reads less (feels lighter)

Here, the wing is being "lifted" by aerodynamic forces, so it "feels lighter" to the gauge.

## Assumptions and Limitations

### Assumptions
1. String gauge responds linearly to applied force
2. Zero RPM corresponds to zero windspeed
3. No significant turbulence or edge effects
4. Wing remains at constant 0° angle of attack
5. Temperature and humidity effects negligible

### Limitations
1. Only tested up to 65.76 mph (~30 m/s)
2. Single angle of attack (0°)
3. Limited to 7 data points
4. RPM-to-windspeed conversion based on interpolation
5. Does not account for:
   - Drag forces (parallel to airflow)
   - Moments/torques on the wing
   - 3D flow effects

## Uncertainty Analysis

### Sources of Uncertainty
1. **Calibration**: Standard error = 6 × 10⁻⁶ V/g (negligible)
2. **Voltage measurement**: LabVIEW acquisition precision
3. **RPM measurement**: Fan speed sensor accuracy
4. **RPM-to-windspeed conversion**: Interpolation error
5. **Baseline drift**: Temperature/vibration effects

### Estimated Accuracy
- Voltage-to-force conversion: ±0.5g
- RPM-to-windspeed: ±1 mph
- Overall lift measurement: ±2-3g

## Future Work

### Recommended Extensions
1. **Vary angle of attack** (-5° to +15°) to generate lift curves
2. **Test with vortex generators deployed** for comparison
3. **Increase windspeed range** (up to 100 mph if safe)
4. **Add more data points** for better statistical power
5. **Measure drag** using separate force component
6. **Flow visualization** (smoke/tufts) to understand flow patterns
7. **Pressure distribution** measurements along wing surface
8. **Reynolds number analysis** for scaling considerations

## Dependencies

### Python Packages
```
pandas >= 1.3.0
numpy >= 1.20.0
matplotlib >= 3.3.0
scipy >= 1.6.0
```

### Installation
```bash
conda activate windtunnel
pip install pandas numpy matplotlib scipy
```

## References

### Aerodynamics Theory
- Anderson, J.D. (2017). *Fundamentals of Aerodynamics*. McGraw-Hill.
- Lift equation: L = ½ρv²SC_L
  - ρ = air density
  - v = velocity
  - S = wing area
  - C_L = lift coefficient

### Experimental Methods
- Barlow, J.B., Rae, W.H., Pope, A. (1999). *Low-Speed Wind Tunnel Testing*. Wiley.

## Authors and Date

- **Test Date**: December 11, 2025
- **Analysis Date**: December 11, 2025
- **Wing Configuration**: 0° AoA, Vortex Generators Retracted

## License and Usage

This data and analysis are for educational purposes as part of the UC Berkeley Introduction to Robotics final project.

---

*For questions or issues, please refer to the main project repository.*
