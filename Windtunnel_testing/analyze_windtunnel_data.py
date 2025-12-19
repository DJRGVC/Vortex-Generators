#!/usr/bin/env python
"""
Wind Tunnel Data Analysis for Wing at 0 Degree AoA (Vortex Generators Retracted)

This script analyzes string gauge voltage data from wind tunnel testing and calculates
lift force as a function of windspeed. The string gauge measures the apparent weight
of the wing - as lift increases, the measured weight decreases.

Methodology:
1. Calibrate voltage-to-force relationship using known weights
2. Convert run data voltages to apparent weights using calibration
3. Calculate lift as: Lift = Baseline_weight - Apparent_weight
4. Plot lift vs windspeed relationship
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from scipy import stats
import json

# Calibration files and known weights (grams)
calibration_files = [
    ('calibration0g.xls', 0),
    ('calibration10g.xls', 10),
    ('calibration20g.xls', 20),
    ('calibration30g.xls', 30),
    ('calibration40g.xls', 40),
    ('calibration50g.xls', 50),
    ('calibration60g.xls', 60),
    ('calibration70g.xls', 70),
    ('calibration80g.xls', 80),
    ('calibration90g.xls', 90),
    ('calibration100g.xls', 100)
]

# RPM to mph conversion data (empirically determined)
rpm_to_mph = {
    790: 15.41,
    1188: 25.05,
    1583: 34.00,
    1984: 45.63,
    2382: 57.26,
    2769: 65.76
}

def read_data_row(filename):
    """
    Read row 23 (data row) from the LabVIEW-generated tab-delimited file.

    File format:
    - Row 22 (header=21): Column names
    - Row 23: Data values

    Returns:
        pandas DataFrame with single row of data
    """
    df = pd.read_csv(filename, sep='\t', header=21, nrows=1)
    return df

def get_calibration_data():
    """
    Extract voltage data from all calibration files.

    Returns:
        tuple: (grams array, voltages array) from calibration measurements
    """
    grams = []
    voltages = []

    for cal_file, weight in calibration_files:
        df = read_data_row(cal_file)
        voltage = df['Untitled 3'].values[0]  # Strain gauge voltage (column 5)
        grams.append(weight)
        voltages.append(voltage)

    return np.array(grams), np.array(voltages)

def get_run_data():
    """
    Extract RPM and voltage data from all run files.

    Returns:
        tuple: (rpms array, voltages array) from experimental runs
    """
    run_files = [f'0deg_run{i}.xls' for i in range(1, 8)]
    rpms = []
    voltages = []

    for run_file in run_files:
        df = read_data_row(run_file)
        rpm = df['Untitled'].values[0]  # Fan RPM (column 2)
        voltage = df['Untitled 3'].values[0]  # Strain gauge voltage (column 5)
        rpms.append(rpm)
        voltages.append(voltage)

    return np.array(rpms), np.array(voltages)

def rpm_to_windspeed(rpm):
    """
    Convert RPM to windspeed (mph) using linear interpolation.

    Special case: 0 RPM = 0 mph
    For other values, interpolate between known RPM-windspeed pairs.

    Args:
        rpm: Fan RPM value or array of values

    Returns:
        Windspeed in mph
    """
    if np.isscalar(rpm):
        if rpm == 0:
            return 0.0
        rpm_array = np.array([rpm])
        scalar_input = True
    else:
        rpm_array = np.array(rpm)
        scalar_input = False

    rpm_keys = np.array(list(rpm_to_mph.keys()))
    mph_values = np.array(list(rpm_to_mph.values()))

    # Interpolate
    windspeed = np.interp(rpm_array, rpm_keys, mph_values)

    # Handle 0 RPM case
    windspeed[rpm_array == 0] = 0.0

    return windspeed[0] if scalar_input else windspeed

def voltage_to_force(voltage, slope, intercept):
    """
    Convert voltage to force (grams) using calibration curve.

    Calibration equation: V = slope * F + intercept
    Inverted: F = (V - intercept) / slope

    Args:
        voltage: Strain gauge voltage reading(s)
        slope: Calibration curve slope (V/g)
        intercept: Calibration curve intercept (V)

    Returns:
        Force in grams (apparent weight on strain gauge)
    """
    return (voltage - intercept) / slope

# ============================================================================
# Main Analysis
# ============================================================================

print("="*70)
print("Wind Tunnel Data Analysis - Lift Force Calculation")
print("Wing at 0 Degree AoA - Vortex Generators Retracted")
print("="*70)

# Step 1: Create calibration curve
print("\n[1] CALIBRATION")
print("-" * 70)
cal_grams, cal_voltages = get_calibration_data()

# Linear regression: V = slope * F + intercept
slope, intercept, r_value, p_value, std_err = stats.linregress(cal_grams, cal_voltages)

print(f"Calibration equation: V = {slope:.6f} * F + {intercept:.6f}")
print(f"Coefficient of determination: R² = {r_value**2:.6f}")
print(f"Standard error: {std_err:.6f} V/g")
print(f"\nCalibration quality: {'Excellent' if r_value**2 > 0.999 else 'Good' if r_value**2 > 0.99 else 'Fair'}")

# Step 2: Process run data
print(f"\n[2] EXPERIMENTAL RUN DATA")
print("-" * 70)
run_rpms, run_voltages = get_run_data()

# Convert to physical units
run_windspeeds = np.array([rpm_to_windspeed(rpm) for rpm in run_rpms])
run_apparent_weights = voltage_to_force(run_voltages, slope, intercept)

# Step 3: Calculate lift force
print(f"\n[3] LIFT FORCE CALCULATION")
print("-" * 70)

# Baseline is the apparent weight at zero windspeed (0 RPM)
baseline_weight = run_apparent_weights[0]
print(f"Baseline weight (0 mph): {baseline_weight:.2f} g")
print(f"\nLift = Baseline_weight - Apparent_weight")
print(f"(As the wing generates lift, it reduces the load on the strain gauge)\n")

# Calculate lift for each run
run_lift = baseline_weight - run_apparent_weights

# Display results table
print(f"{'Run':<6}{'RPM':<10}{'Windspeed':<12}{'Voltage':<12}{'Apparent':<12}{'Lift':<10}")
print(f"{'#':<6}{'(RPM)':<10}{'(mph)':<12}{'(V)':<12}{'Weight (g)':<12}{'(g)':<10}")
print("-" * 70)
for i, (rpm, ws, v, aw, lift) in enumerate(zip(run_rpms, run_windspeeds,
                                                run_voltages, run_apparent_weights,
                                                run_lift), 1):
    print(f"{i:<6}{rpm:<10.1f}{ws:<12.2f}{v:<12.5f}{aw:<12.2f}{lift:<10.2f}")

# Step 4: Statistical analysis of lift vs windspeed
print(f"\n[4] LIFT VS WINDSPEED ANALYSIS")
print("-" * 70)

# Include ALL data points (including 0,0) for curve fitting
# This ensures the fit passes through the origin, which is physically correct
# (zero windspeed should give zero lift)

# Fit quadratic model: Lift = a*v² + b*v + c
# (Aerodynamic forces typically scale with velocity squared)
coeffs_quad = np.polyfit(run_windspeeds, run_lift, 2)
poly_quad = np.poly1d(coeffs_quad)

# Calculate R² for quadratic fit
lift_pred_quad = poly_quad(run_windspeeds)
ss_res = np.sum((run_lift - lift_pred_quad)**2)
ss_tot = np.sum((run_lift - np.mean(run_lift))**2)
r2_quad = 1 - (ss_res / ss_tot)

print(f"Quadratic fit (all points): Lift = {coeffs_quad[0]:.4f}*v² + {coeffs_quad[1]:.3f}*v + {coeffs_quad[2]:.2f}")
print(f"R² = {r2_quad:.6f}")

# For comparison, also compute nonzero mask for plotting
nonzero_mask = run_windspeeds > 0
ws_nonzero = run_windspeeds[nonzero_mask]
lift_nonzero = run_lift[nonzero_mask]

# Also fit linear model for comparison (using all points)
coeffs_lin = np.polyfit(run_windspeeds, run_lift, 1)
poly_lin = np.poly1d(coeffs_lin)
lift_pred_lin = poly_lin(run_windspeeds)
ss_res_lin = np.sum((run_lift - lift_pred_lin)**2)
r2_lin = 1 - (ss_res_lin / ss_tot)

print(f"\nLinear fit (all points): Lift = {coeffs_lin[0]:.3f}*v + {coeffs_lin[1]:.2f}")
print(f"R² = {r2_lin:.6f}")

# Save results to JSON
results = {
    'calibration': {
        'slope': float(slope),
        'intercept': float(intercept),
        'r_squared': float(r_value**2),
        'std_error': float(std_err)
    },
    'baseline_weight_g': float(baseline_weight),
    'runs': [
        {
            'run_number': int(i),
            'rpm': float(rpm),
            'windspeed_mph': float(ws),
            'voltage_v': float(v),
            'apparent_weight_g': float(aw),
            'lift_g': float(lift)
        }
        for i, (rpm, ws, v, aw, lift) in enumerate(zip(run_rpms, run_windspeeds,
                                                        run_voltages, run_apparent_weights,
                                                        run_lift), 1)
    ],
    'fits': {
        'quadratic': {
            'coefficients': [float(c) for c in coeffs_quad],
            'r_squared': float(r2_quad)
        },
        'linear': {
            'coefficients': [float(c) for c in coeffs_lin],
            'r_squared': float(r2_lin)
        }
    }
}

with open('analysis_results.json', 'w') as f:
    json.dump(results, f, indent=2)

print(f"\nResults saved to 'analysis_results.json'")

# ============================================================================
# Visualization
# ============================================================================

print(f"\n[5] GENERATING PLOTS")
print("-" * 70)

# Figure 1: Calibration curve
fig1, ax1 = plt.subplots(figsize=(10, 7))
ax1.scatter(cal_grams, cal_voltages, s=100, alpha=0.7, color='steelblue',
           edgecolors='navy', linewidth=1.5, label='Calibration Data', zorder=3)

# Plot calibration fit
grams_fit = np.linspace(0, 100, 100)
voltages_fit = slope * grams_fit + intercept
ax1.plot(grams_fit, voltages_fit, 'r--', linewidth=2.5, alpha=0.8,
         label=f'Linear Fit: V = {slope:.5f}F + {intercept:.4f}\nR² = {r_value**2:.6f}',
         zorder=2)

ax1.set_xlabel('Applied Force (grams)', fontsize=13, fontweight='bold')
ax1.set_ylabel('Strain Gauge Voltage (V)', fontsize=13, fontweight='bold')
ax1.set_title('String Gauge Calibration Curve', fontsize=15, fontweight='bold', pad=15)
ax1.legend(fontsize=11, loc='lower right')
ax1.grid(True, alpha=0.3, linestyle='--')
ax1.set_axisbelow(True)

plt.tight_layout()
plt.savefig('calibration_curve.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: calibration_curve.png")

# Figure 2: Lift vs Windspeed (main result) - UC Berkeley themed
fig2, ax2 = plt.subplots(figsize=(12, 8))

# UC Berkeley colors
berkeley_blue = '#003262'
california_gold = '#FDB515'
light_gray = '#F5F5F5'

# Set clean background
ax2.set_facecolor('white')
fig2.patch.set_facecolor('white')

# Plot quadratic fit extending to 0 mph
ws_smooth = np.linspace(0, ws_nonzero.max(), 200)
lift_smooth = poly_quad(ws_smooth)
ax2.plot(ws_smooth, lift_smooth, '-', color=berkeley_blue, linewidth=3, alpha=0.85,
         label=f'Quadratic Fit: {coeffs_quad[0]:.4f}v² + {coeffs_quad[1]:.3f}v + {coeffs_quad[2]:.2f}\nR² = {r2_quad:.4f}',
         zorder=3)

# Plot measured data points
ax2.scatter(run_windspeeds, run_lift, s=180, alpha=0.95, color=california_gold,
           marker='o', edgecolors=berkeley_blue, linewidth=2.5, label='Measured Data',
           zorder=5)

# Annotate data points (upper left of dots)
for rpm, ws, lift in zip(run_rpms, run_windspeeds, run_lift):
    if ws > 0:  # Skip zero windspeed point
        ax2.annotate(f'{lift:.1f}g\n{ws:.1f} mph',
                    xy=(ws, lift),
                    xytext=(-15, 12),
                    textcoords='offset points',
                    fontsize=9,
                    ha='right',
                    bbox=dict(boxstyle='round,pad=0.35', facecolor='white',
                             edgecolor=berkeley_blue, alpha=0.9, linewidth=1),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.15',
                                  color=berkeley_blue, lw=1.2, alpha=0.7),
                    zorder=4)

ax2.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax2.set_xlabel('Windspeed (mph)', fontsize=14, fontweight='bold', color=berkeley_blue)
ax2.set_ylabel('Lift Force (grams)', fontsize=14, fontweight='bold', color=berkeley_blue)
ax2.set_title('Lift Force vs Windspeed\nWing at 0° Angle of Attack - Vortex Generators Retracted',
             fontsize=16, fontweight='bold', pad=20, color=berkeley_blue)
ax2.legend(fontsize=11, loc='upper left', framealpha=0.95, edgecolor=berkeley_blue, fancybox=False)
ax2.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax2.set_axisbelow(True)

# Set axis limits with some padding
ax2.set_xlim(-2, ws_nonzero.max() + 5)
ax2.set_ylim(min(run_lift) - 20, max(run_lift) + 50)

# Cleaner spines
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.spines['left'].set_color(berkeley_blue)
ax2.spines['bottom'].set_color(berkeley_blue)
ax2.spines['left'].set_linewidth(1.5)
ax2.spines['bottom'].set_linewidth(1.5)
ax2.tick_params(colors=berkeley_blue, which='both', labelsize=11)

plt.tight_layout()
plt.savefig('lift_vs_windspeed.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_vs_windspeed.png")

# Figure 3: Combined view - Apparent weight and lift
fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(12, 10))

# Top panel: Apparent weight vs windspeed
ax3a.scatter(run_windspeeds, run_apparent_weights, s=120, alpha=0.7,
            color='forestgreen', marker='s', edgecolors='darkgreen', linewidth=1.5)
ax3a.plot(run_windspeeds, run_apparent_weights, '--', alpha=0.4, color='gray', linewidth=1.5)
ax3a.axhline(y=baseline_weight, color='red', linestyle='--', linewidth=2,
            alpha=0.7, label=f'Baseline ({baseline_weight:.1f}g)')
ax3a.set_xlabel('Windspeed (mph)', fontsize=12, fontweight='bold')
ax3a.set_ylabel('Apparent Weight (grams)', fontsize=12, fontweight='bold')
ax3a.set_title('Apparent Weight on String Gauge vs Windspeed', fontsize=14, fontweight='bold')
ax3a.legend(fontsize=11)
ax3a.grid(True, alpha=0.3)

# Bottom panel: Lift vs windspeed
ax3b.scatter(run_windspeeds, run_lift, s=120, alpha=0.7,
            color='crimson', marker='o', edgecolors='darkred', linewidth=1.5)
if len(ws_nonzero) > 0:
    ax3b.plot(ws_smooth, lift_smooth, 'b-', linewidth=2.5, alpha=0.7)
ax3b.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.3)
ax3b.set_xlabel('Windspeed (mph)', fontsize=12, fontweight='bold')
ax3b.set_ylabel('Lift Force (grams)', fontsize=12, fontweight='bold')
ax3b.set_title('Calculated Lift Force vs Windspeed', fontsize=14, fontweight='bold')
ax3b.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('combined_analysis.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: combined_analysis.png")

print("\n" + "="*70)
print("Analysis Complete!")
print("="*70)
print(f"\nGenerated files:")
print(f"  - calibration_curve.png")
print(f"  - lift_vs_windspeed.png")
print(f"  - combined_analysis.png")
print(f"  - analysis_results.json")
