#!/usr/bin/env python
"""
Comprehensive Wind Tunnel Analysis - All Configurations
Analyzes lift and drag for deployed/retracted vortex generators at 0/10/20 degree angles of attack
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import stats
import json
import os

# ============================================================================
# Configuration and Constants
# ============================================================================

# Calibration parameters (from previous analysis)
CALIBRATION_SLOPE = -0.001857
CALIBRATION_INTERCEPT = -1.598511

# RPM to mph conversion data (empirically determined)
rpm_to_mph = {
    790: 15.41,
    1188: 25.05,
    1583: 34.00,
    1984: 45.63,
    2382: 57.26,
    2769: 65.76
}

# Drag-to-lift ratios for different angles of attack (typical airfoil values)
# These are estimates based on typical airfoil performance
DRAG_LIFT_RATIOS = {
    0: 0.04,    # Low drag at zero angle of attack
    10: 0.08,   # Moderate drag at 10 degrees
    20: 0.15    # Higher drag at 20 degrees (approaching stall)
}

# UC Berkeley colors
BERKELEY_BLUE = '#003262'
CALIFORNIA_GOLD = '#FDB515'
FOUNDERS_ROCK = '#3B7EA1'
MEDALIST = '#C4820E'

# Color schemes for each angle
ANGLE_COLORS = {
    0: '#003262',   # Berkeley Blue for 0 degrees
    10: '#3B7EA1',  # Founders Rock for 10 degrees
    20: '#FDB515'   # California Gold for 20 degrees
}

# ============================================================================
# Helper Functions
# ============================================================================

def read_data_row(filename):
    """Read row 23 (data row) from the LabVIEW-generated tab-delimited file."""
    df = pd.read_csv(filename, sep='\t', header=21, nrows=1)
    return df

def rpm_to_windspeed(rpm):
    """Convert RPM to windspeed (mph) using linear interpolation."""
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

    windspeed = np.interp(rpm_array, rpm_keys, mph_values)
    windspeed[rpm_array == 0] = 0.0

    return windspeed[0] if scalar_input else windspeed

def voltage_to_force(voltage, slope, intercept):
    """Convert voltage to force (grams) using calibration curve."""
    return (voltage - intercept) / slope

def decompose_forces(vertical_force, angle_deg):
    """
    Decompose measured vertical aerodynamic force into lift and drag components.

    At angle of attack α:
    - Lift L is perpendicular to airflow
    - Drag D is parallel to airflow
    - Vertical component measured: L*cos(α) - D*sin(α)

    Using assumed D/L ratio:
    L = vertical_force / (cos(α) - (D/L)*sin(α))
    D = (D/L) * L
    """
    alpha_rad = np.radians(angle_deg)
    dl_ratio = DRAG_LIFT_RATIOS.get(angle_deg, 0.08)  # Default to moderate drag

    # Handle array or scalar input
    cos_a = np.cos(alpha_rad)
    sin_a = np.sin(alpha_rad)

    # Calculate lift from vertical force measurement
    # vertical_force = L*cos(α) - D*sin(α) = L*(cos(α) - (D/L)*sin(α))
    denominator = cos_a - dl_ratio * sin_a
    lift = vertical_force / denominator
    drag = dl_ratio * lift

    return lift, drag

def read_configuration_data(config_name, angle_deg, vg_state):
    """
    Read data for a specific configuration.

    Args:
        config_name: Directory name (e.g., 'deployed__00deg')
        angle_deg: Angle of attack in degrees
        vg_state: 'deployed' or 'retracted'

    Returns:
        dict with rpms, voltages, windspeeds, lift, drag
    """
    base_dir = 'wind tunnel data'
    config_dir = os.path.join(base_dir, config_name)

    # Determine file pattern
    if config_name == 'retracted__00deg':
        file_pattern = 'retracted_00_{:02d}.xls'
    elif config_name == 'deployed__00deg':
        file_pattern = 'deployed_{:02d}.xls'
    elif config_name == 'retracted_10deg':
        file_pattern = 'retracted_10_{:02d}.xls'
    elif config_name == 'deployed_10deg':
        file_pattern = 'deployed_10_{:02d}.xls'
    elif config_name == 'retracted_20deg':
        file_pattern = 'retracted_20_{:02d}.xls'
    elif config_name == 'deployed__20deg(last3bad)':
        file_pattern = 'deployed_20_{:02d}.xls'
    else:
        raise ValueError(f"Unknown configuration: {config_name}")

    # Determine file range (exclude last 3 for deployed 20deg)
    if config_name == 'deployed__20deg(last3bad)':
        file_indices = range(1, 4)  # Only files 01, 02, 03 (04-06 are bad, 00 missing)
    else:
        file_indices = range(0, 7)  # Files 00-06

    rpms = []
    voltages = []

    for i in file_indices:
        filename = os.path.join(config_dir, file_pattern.format(i))
        if os.path.exists(filename):
            df = read_data_row(filename)
            rpm = df['Untitled'].values[0]
            voltage = df['Untitled 3'].values[0]
            rpms.append(rpm)
            voltages.append(voltage)

    rpms = np.array(rpms)
    voltages = np.array(voltages)

    # Convert to physical units
    windspeeds = np.array([rpm_to_windspeed(rpm) for rpm in rpms])
    apparent_weights = voltage_to_force(voltages, CALIBRATION_SLOPE, CALIBRATION_INTERCEPT)

    # Calculate baseline and vertical aerodynamic force
    baseline_weight = apparent_weights[0]
    vertical_aero_force = baseline_weight - apparent_weights

    # Decompose into lift and drag
    lift, drag = decompose_forces(vertical_aero_force, angle_deg)

    return {
        'rpms': rpms,
        'voltages': voltages,
        'windspeeds': windspeeds,
        'apparent_weights': apparent_weights,
        'baseline_weight': baseline_weight,
        'vertical_force': vertical_aero_force,
        'lift': lift,
        'drag': drag,
        'angle': angle_deg,
        'vg_state': vg_state
    }

# ============================================================================
# Main Analysis
# ============================================================================

print("="*70)
print("Comprehensive Wind Tunnel Analysis")
print("Lift and Drag for All Configurations")
print("="*70)

# Define all configurations to analyze
configurations = [
    ('retracted__00deg', 0, 'retracted'),
    ('deployed__00deg', 0, 'deployed'),
    ('retracted_10deg', 10, 'retracted'),
    ('deployed_10deg', 10, 'deployed'),
    ('retracted_20deg', 20, 'retracted'),
    ('deployed__20deg(last3bad)', 20, 'deployed'),
]

# Load all data
all_data = {}
for config_name, angle, vg_state in configurations:
    print(f"\nLoading {vg_state} {angle}° data...")
    key = (vg_state, angle)
    try:
        all_data[key] = read_configuration_data(config_name, angle, vg_state)
        print(f"  ✓ Loaded {len(all_data[key]['rpms'])} data points")
        print(f"    Baseline weight: {all_data[key]['baseline_weight']:.2f}g")
        print(f"    Max lift: {np.max(all_data[key]['lift']):.2f}g at {np.max(all_data[key]['windspeeds']):.2f} mph")
        print(f"    Max drag: {np.max(all_data[key]['drag']):.2f}g at {np.max(all_data[key]['windspeeds']):.2f} mph")
    except Exception as e:
        print(f"  ✗ Error loading data: {e}")

# ============================================================================
# Generate Plots
# ============================================================================

print(f"\n{'='*70}")
print("Generating Plots")
print("="*70)

# Plot 1: Lift vs Windspeed - Retracted
fig1, ax1 = plt.subplots(figsize=(12, 8))
ax1.set_facecolor('white')
fig1.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('retracted', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        lift = data['lift']

        # Plot data points
        ax1.scatter(ws, lift, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='o', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

        # Fit and plot curve (quadratic)
        if len(ws) > 2:
            coeffs = np.polyfit(ws, lift, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws.max(), 100)
            lift_smooth = poly(ws_smooth)
            ax1.plot(ws_smooth, lift_smooth, '-', color=ANGLE_COLORS[angle],
                    linewidth=2.5, alpha=0.7, zorder=3)

ax1.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax1.set_xlabel('Windspeed (mph)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax1.set_ylabel('Lift Force (grams)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax1.set_title('Lift Force vs Windspeed - Vortex Generators Retracted',
             fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax1.legend(fontsize=12, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE)
ax1.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax1.set_axisbelow(True)
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.spines['left'].set_color(BERKELEY_BLUE)
ax1.spines['bottom'].set_color(BERKELEY_BLUE)
ax1.spines['left'].set_linewidth(1.5)
ax1.spines['bottom'].set_linewidth(1.5)
ax1.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)
plt.tight_layout()
plt.savefig('lift_retracted_all_angles.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_retracted_all_angles.png")
plt.close()

# Plot 2: Lift vs Windspeed - Deployed
fig2, ax2 = plt.subplots(figsize=(12, 8))
ax2.set_facecolor('white')
fig2.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('deployed', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        lift = data['lift']

        # Plot data points
        ax2.scatter(ws, lift, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='s', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

        # Fit and plot curve
        if len(ws) > 2:
            coeffs = np.polyfit(ws, lift, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws.max(), 100)
            lift_smooth = poly(ws_smooth)
            ax2.plot(ws_smooth, lift_smooth, '-', color=ANGLE_COLORS[angle],
                    linewidth=2.5, alpha=0.7, zorder=3)

ax2.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax2.set_xlabel('Windspeed (mph)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax2.set_ylabel('Lift Force (grams)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax2.set_title('Lift Force vs Windspeed - Vortex Generators Deployed',
             fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax2.legend(fontsize=12, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE)
ax2.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax2.set_axisbelow(True)
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.spines['left'].set_color(BERKELEY_BLUE)
ax2.spines['bottom'].set_color(BERKELEY_BLUE)
ax2.spines['left'].set_linewidth(1.5)
ax2.spines['bottom'].set_linewidth(1.5)
ax2.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)
plt.tight_layout()
plt.savefig('lift_deployed_all_angles.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_deployed_all_angles.png")
plt.close()

# Plot 3: Drag vs Windspeed - Retracted
fig3, ax3 = plt.subplots(figsize=(12, 8))
ax3.set_facecolor('white')
fig3.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('retracted', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        drag = data['drag']

        # Plot data points
        ax3.scatter(ws, drag, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='o', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

        # Fit and plot curve
        if len(ws) > 2:
            coeffs = np.polyfit(ws, drag, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws.max(), 100)
            drag_smooth = poly(ws_smooth)
            ax3.plot(ws_smooth, drag_smooth, '-', color=ANGLE_COLORS[angle],
                    linewidth=2.5, alpha=0.7, zorder=3)

ax3.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax3.set_xlabel('Windspeed (mph)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax3.set_ylabel('Drag Force (grams)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax3.set_title('Drag Force vs Windspeed - Vortex Generators Retracted',
             fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax3.legend(fontsize=12, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE)
ax3.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax3.set_axisbelow(True)
ax3.spines['top'].set_visible(False)
ax3.spines['right'].set_visible(False)
ax3.spines['left'].set_color(BERKELEY_BLUE)
ax3.spines['bottom'].set_color(BERKELEY_BLUE)
ax3.spines['left'].set_linewidth(1.5)
ax3.spines['bottom'].set_linewidth(1.5)
ax3.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)
plt.tight_layout()
plt.savefig('drag_retracted_all_angles.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: drag_retracted_all_angles.png")
plt.close()

# Plot 4: Drag vs Windspeed - Deployed
fig4, ax4 = plt.subplots(figsize=(12, 8))
ax4.set_facecolor('white')
fig4.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('deployed', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        drag = data['drag']

        # Plot data points
        ax4.scatter(ws, drag, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='s', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

        # Fit and plot curve
        if len(ws) > 2:
            coeffs = np.polyfit(ws, drag, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws.max(), 100)
            drag_smooth = poly(ws_smooth)
            ax4.plot(ws_smooth, drag_smooth, '-', color=ANGLE_COLORS[angle],
                    linewidth=2.5, alpha=0.7, zorder=3)

ax4.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax4.set_xlabel('Windspeed (mph)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax4.set_ylabel('Drag Force (grams)', fontsize=14, fontweight='bold', color=BERKELEY_BLUE)
ax4.set_title('Drag Force vs Windspeed - Vortex Generators Deployed',
             fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax4.legend(fontsize=12, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE)
ax4.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax4.set_axisbelow(True)
ax4.spines['top'].set_visible(False)
ax4.spines['right'].set_visible(False)
ax4.spines['left'].set_color(BERKELEY_BLUE)
ax4.spines['bottom'].set_color(BERKELEY_BLUE)
ax4.spines['left'].set_linewidth(1.5)
ax4.spines['bottom'].set_linewidth(1.5)
ax4.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)
plt.tight_layout()
plt.savefig('drag_deployed_all_angles.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: drag_deployed_all_angles.png")
plt.close()

# ============================================================================
# Save Summary Data
# ============================================================================

summary = {}
for key, data in all_data.items():
    vg_state, angle = key
    summary[f"{vg_state}_{angle}deg"] = {
        'angle_deg': angle,
        'vg_state': vg_state,
        'baseline_weight_g': float(data['baseline_weight']),
        'max_windspeed_mph': float(np.max(data['windspeeds'])),
        'max_lift_g': float(np.max(data['lift'])),
        'max_drag_g': float(np.max(data['drag'])),
        'num_points': len(data['windspeeds'])
    }

with open('comprehensive_analysis_summary.json', 'w') as f:
    json.dump(summary, f, indent=2)

print("\n" + "="*70)
print("Analysis Complete!")
print("="*70)
print("\nGenerated files:")
print("  - lift_retracted_all_angles.png")
print("  - lift_deployed_all_angles.png")
print("  - drag_retracted_all_angles.png")
print("  - drag_deployed_all_angles.png")
print("  - comprehensive_analysis_summary.json")
print()
