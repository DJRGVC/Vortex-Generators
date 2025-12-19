#!/usr/bin/env python
"""
Complete Wind Tunnel Analysis - All Configurations Including Bad Data
Generates 6 plots: 4 individual (with bad data) + 2 comprehensive comparison plots
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from scipy import stats
import json
import os

# ============================================================================
# Configuration and Constants
# ============================================================================

# Calibration parameters (from previous analysis)
CALIBRATION_SLOPE = -0.001857
CALIBRATION_INTERCEPT = -1.598511

# RPM to mph conversion data
rpm_to_mph = {
    790: 15.41,
    1188: 25.05,
    1583: 34.00,
    1984: 45.63,
    2382: 57.26,
    2769: 65.76
}

# Drag-to-lift ratios for different angles of attack
DRAG_LIFT_RATIOS = {
    0: 0.04,
    10: 0.08,
    20: 0.15
}

# UC Berkeley colors
BERKELEY_BLUE = '#003262'
CALIFORNIA_GOLD = '#FDB515'
FOUNDERS_ROCK = '#3B7EA1'
MEDALIST = '#C4820E'
LAWRENCE = '#00B0DA'
LAP_LANE = '#00A598'

# Color schemes for each angle
ANGLE_COLORS = {
    0: '#003262',   # Berkeley Blue
    10: '#3B7EA1',  # Founders Rock
    20: '#FDB515'   # California Gold
}

# Marker styles
RETRACTED_MARKER = 'o'
DEPLOYED_MARKER = 's'

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
    """Decompose measured vertical aerodynamic force into lift and drag components."""
    alpha_rad = np.radians(angle_deg)
    dl_ratio = DRAG_LIFT_RATIOS.get(angle_deg, 0.08)

    cos_a = np.cos(alpha_rad)
    sin_a = np.sin(alpha_rad)

    denominator = cos_a - dl_ratio * sin_a
    lift = vertical_force / denominator
    drag = dl_ratio * lift

    return lift, drag

def read_configuration_data(config_name, angle_deg, vg_state, include_bad=False):
    """
    Read data for a specific configuration.

    Args:
        config_name: Directory name
        angle_deg: Angle of attack in degrees
        vg_state: 'deployed' or 'retracted'
        include_bad: If True, include all data even if marked as bad
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

    # Determine file range
    if config_name == 'deployed__20deg(last3bad)' and not include_bad:
        file_indices = range(1, 4)  # Only files 01, 02, 03
    elif config_name == 'deployed__20deg(last3bad)' and include_bad:
        file_indices = range(1, 7)  # Files 01-06 (all available, 00 missing)
    else:
        file_indices = range(0, 7)

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
print("Complete Wind Tunnel Analysis - Including All Data")
print("="*70)

# Define all configurations
configurations = [
    ('retracted__00deg', 0, 'retracted'),
    ('deployed__00deg', 0, 'deployed'),
    ('retracted_10deg', 10, 'retracted'),
    ('deployed_10deg', 10, 'deployed'),
    ('retracted_20deg', 20, 'retracted'),
    ('deployed__20deg(last3bad)', 20, 'deployed'),
]

# Load all data (including bad data)
all_data = {}
for config_name, angle, vg_state in configurations:
    print(f"\nLoading {vg_state} {angle}° data (including all points)...")
    key = (vg_state, angle)
    try:
        all_data[key] = read_configuration_data(config_name, angle, vg_state, include_bad=True)
        print(f"  ✓ Loaded {len(all_data[key]['rpms'])} data points")
        print(f"    Max lift: {np.max(all_data[key]['lift']):.2f}g at {np.max(all_data[key]['windspeeds']):.2f} mph")
    except Exception as e:
        print(f"  ✗ Error: {e}")

# ============================================================================
# Plots 1-4: Individual Plots with All Data
# ============================================================================

print(f"\n{'='*70}")
print("Generating Individual Plots (with all data)")
print("="*70)

# Plot 1: Lift - Retracted
fig1, ax1 = plt.subplots(figsize=(12, 8))
ax1.set_facecolor('white')
fig1.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('retracted', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        lift = data['lift']

        ax1.scatter(ws, lift, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='o', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

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
ax1.set_title('Lift Force vs Windspeed - Vortex Generators Retracted\n(All Data Points)',
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
plt.savefig('lift_retracted_all_data.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_retracted_all_data.png")
plt.close()

# Plot 2: Lift - Deployed
fig2, ax2 = plt.subplots(figsize=(12, 8))
ax2.set_facecolor('white')
fig2.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('deployed', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        lift = data['lift']

        ax2.scatter(ws, lift, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='s', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

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
ax2.set_title('Lift Force vs Windspeed - Vortex Generators Deployed\n(All Data Points)',
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
plt.savefig('lift_deployed_all_data.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_deployed_all_data.png")
plt.close()

# Plot 3: Drag - Retracted
fig3, ax3 = plt.subplots(figsize=(12, 8))
ax3.set_facecolor('white')
fig3.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('retracted', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        drag = data['drag']

        ax3.scatter(ws, drag, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='o', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

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
ax3.set_title('Drag Force vs Windspeed - Vortex Generators Retracted\n(All Data Points)',
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
plt.savefig('drag_retracted_all_data.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: drag_retracted_all_data.png")
plt.close()

# Plot 4: Drag - Deployed
fig4, ax4 = plt.subplots(figsize=(12, 8))
ax4.set_facecolor('white')
fig4.patch.set_facecolor('white')

for angle in [0, 10, 20]:
    key = ('deployed', angle)
    if key in all_data:
        data = all_data[key]
        ws = data['windspeeds']
        drag = data['drag']

        ax4.scatter(ws, drag, s=120, alpha=0.85, color=ANGLE_COLORS[angle],
                   marker='s', edgecolors='black', linewidth=1.5,
                   label=f'{angle}° AoA', zorder=5)

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
ax4.set_title('Drag Force vs Windspeed - Vortex Generators Deployed\n(All Data Points)',
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
plt.savefig('drag_deployed_all_data.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: drag_deployed_all_data.png")
plt.close()

# ============================================================================
# Plot 5: Comprehensive Lift Comparison with Delta
# ============================================================================

print(f"\n{'='*70}")
print("Generating Comprehensive Comparison Plots")
print("="*70)

fig5, (ax5_main, ax5_delta) = plt.subplots(2, 1, figsize=(14, 12),
                                            gridspec_kw={'height_ratios': [3, 1]})
fig5.patch.set_facecolor('white')

# Main plot - All 6 configurations
ax5_main.set_facecolor('white')

for angle in [0, 10, 20]:
    # Retracted
    key_r = ('retracted', angle)
    if key_r in all_data:
        data_r = all_data[key_r]
        ws_r = data_r['windspeeds']
        lift_r = data_r['lift']

        ax5_main.scatter(ws_r, lift_r, s=140, alpha=0.85, color=ANGLE_COLORS[angle],
                        marker='o', edgecolors='black', linewidth=2,
                        label=f'{angle}° Retracted', zorder=5)

        if len(ws_r) > 2:
            coeffs = np.polyfit(ws_r, lift_r, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws_r.max(), 100)
            lift_smooth = poly(ws_smooth)
            ax5_main.plot(ws_smooth, lift_smooth, '-', color=ANGLE_COLORS[angle],
                         linewidth=2.5, alpha=0.6, zorder=3)

    # Deployed
    key_d = ('deployed', angle)
    if key_d in all_data:
        data_d = all_data[key_d]
        ws_d = data_d['windspeeds']
        lift_d = data_d['lift']

        ax5_main.scatter(ws_d, lift_d, s=140, alpha=0.85, color=ANGLE_COLORS[angle],
                        marker='s', edgecolors='black', linewidth=2,
                        label=f'{angle}° Deployed', zorder=5)

        if len(ws_d) > 2:
            coeffs = np.polyfit(ws_d, lift_d, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws_d.max(), 100)
            lift_smooth = poly(ws_smooth)
            ax5_main.plot(ws_smooth, lift_smooth, '--', color=ANGLE_COLORS[angle],
                         linewidth=2.5, alpha=0.6, zorder=3)

ax5_main.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax5_main.set_xlabel('Windspeed (mph)', fontsize=13, fontweight='bold', color=BERKELEY_BLUE)
ax5_main.set_ylabel('Lift Force (grams)', fontsize=13, fontweight='bold', color=BERKELEY_BLUE)
ax5_main.set_title('Comprehensive Lift Comparison - All Configurations\nCircles = Retracted | Squares = Deployed',
                   fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax5_main.legend(fontsize=10, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE, ncol=2)
ax5_main.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax5_main.set_axisbelow(True)
ax5_main.spines['top'].set_visible(False)
ax5_main.spines['right'].set_visible(False)
ax5_main.spines['left'].set_color(BERKELEY_BLUE)
ax5_main.spines['bottom'].set_color(BERKELEY_BLUE)
ax5_main.spines['left'].set_linewidth(1.5)
ax5_main.spines['bottom'].set_linewidth(1.5)
ax5_main.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)

# Delta subplot - showing improvement
ax5_delta.set_facecolor('#F8F8F8')

for angle in [0, 10, 20]:
    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        data_r = all_data[key_r]
        data_d = all_data[key_d]

        # Find common windspeeds (approximately)
        max_lift_r = np.max(data_r['lift'])
        max_lift_d = np.max(data_d['lift'])
        delta_lift = max_lift_d - max_lift_r
        percent_change = (delta_lift / max_lift_r * 100) if max_lift_r != 0 else 0

        # Bar chart showing delta
        bar = ax5_delta.bar(angle, percent_change, color=ANGLE_COLORS[angle],
                           alpha=0.7, edgecolor='black', linewidth=1.5, width=3)

        # Annotate bars
        y_pos = percent_change + (2 if percent_change > 0 else -2)
        va = 'bottom' if percent_change > 0 else 'top'
        ax5_delta.text(angle, y_pos, f'{percent_change:+.1f}%',
                      ha='center', va=va, fontsize=11, fontweight='bold',
                      color=ANGLE_COLORS[angle])

ax5_delta.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
ax5_delta.set_xlabel('Angle of Attack (degrees)', fontsize=12, fontweight='bold', color=BERKELEY_BLUE)
ax5_delta.set_ylabel('Δ Lift (%)', fontsize=12, fontweight='bold', color=BERKELEY_BLUE)
ax5_delta.set_title('Lift Change: Deployed vs Retracted (at max windspeed)',
                   fontsize=13, fontweight='bold', color=BERKELEY_BLUE, pad=10)
ax5_delta.set_xticks([0, 10, 20])
ax5_delta.set_xticklabels(['0°', '10°', '20°'])
ax5_delta.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA', axis='y')
ax5_delta.set_axisbelow(True)
ax5_delta.spines['top'].set_visible(False)
ax5_delta.spines['right'].set_visible(False)
ax5_delta.spines['left'].set_color(BERKELEY_BLUE)
ax5_delta.spines['bottom'].set_color(BERKELEY_BLUE)
ax5_delta.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=10)

plt.tight_layout()
plt.savefig('lift_comprehensive_comparison.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: lift_comprehensive_comparison.png")
plt.close()

# ============================================================================
# Plot 6: Comprehensive Drag Comparison with Delta
# ============================================================================

fig6, (ax6_main, ax6_delta) = plt.subplots(2, 1, figsize=(14, 12),
                                            gridspec_kw={'height_ratios': [3, 1]})
fig6.patch.set_facecolor('white')

# Main plot - All 6 configurations
ax6_main.set_facecolor('white')

for angle in [0, 10, 20]:
    # Retracted
    key_r = ('retracted', angle)
    if key_r in all_data:
        data_r = all_data[key_r]
        ws_r = data_r['windspeeds']
        drag_r = data_r['drag']

        ax6_main.scatter(ws_r, drag_r, s=140, alpha=0.85, color=ANGLE_COLORS[angle],
                        marker='o', edgecolors='black', linewidth=2,
                        label=f'{angle}° Retracted', zorder=5)

        if len(ws_r) > 2:
            coeffs = np.polyfit(ws_r, drag_r, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws_r.max(), 100)
            drag_smooth = poly(ws_smooth)
            ax6_main.plot(ws_smooth, drag_smooth, '-', color=ANGLE_COLORS[angle],
                         linewidth=2.5, alpha=0.6, zorder=3)

    # Deployed
    key_d = ('deployed', angle)
    if key_d in all_data:
        data_d = all_data[key_d]
        ws_d = data_d['windspeeds']
        drag_d = data_d['drag']

        ax6_main.scatter(ws_d, drag_d, s=140, alpha=0.85, color=ANGLE_COLORS[angle],
                        marker='s', edgecolors='black', linewidth=2,
                        label=f'{angle}° Deployed', zorder=5)

        if len(ws_d) > 2:
            coeffs = np.polyfit(ws_d, drag_d, 2)
            poly = np.poly1d(coeffs)
            ws_smooth = np.linspace(0, ws_d.max(), 100)
            drag_smooth = poly(ws_smooth)
            ax6_main.plot(ws_smooth, drag_smooth, '--', color=ANGLE_COLORS[angle],
                         linewidth=2.5, alpha=0.6, zorder=3)

ax6_main.axhline(y=0, color='#CCCCCC', linestyle='-', linewidth=1, alpha=0.5)
ax6_main.set_xlabel('Windspeed (mph)', fontsize=13, fontweight='bold', color=BERKELEY_BLUE)
ax6_main.set_ylabel('Drag Force (grams)', fontsize=13, fontweight='bold', color=BERKELEY_BLUE)
ax6_main.set_title('Comprehensive Drag Comparison - All Configurations\nCircles = Retracted | Squares = Deployed',
                   fontsize=16, fontweight='bold', pad=20, color=BERKELEY_BLUE)
ax6_main.legend(fontsize=10, loc='upper left', framealpha=0.95, edgecolor=BERKELEY_BLUE, ncol=2)
ax6_main.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA')
ax6_main.set_axisbelow(True)
ax6_main.spines['top'].set_visible(False)
ax6_main.spines['right'].set_visible(False)
ax6_main.spines['left'].set_color(BERKELEY_BLUE)
ax6_main.spines['bottom'].set_color(BERKELEY_BLUE)
ax6_main.spines['left'].set_linewidth(1.5)
ax6_main.spines['bottom'].set_linewidth(1.5)
ax6_main.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=11)

# Delta subplot
ax6_delta.set_facecolor('#F8F8F8')

for angle in [0, 10, 20]:
    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        data_r = all_data[key_r]
        data_d = all_data[key_d]

        max_drag_r = np.max(data_r['drag'])
        max_drag_d = np.max(data_d['drag'])
        delta_drag = max_drag_d - max_drag_r
        percent_change = (delta_drag / max_drag_r * 100) if max_drag_r != 0 else 0

        bar = ax6_delta.bar(angle, percent_change, color=ANGLE_COLORS[angle],
                           alpha=0.7, edgecolor='black', linewidth=1.5, width=3)

        y_pos = percent_change + (2 if percent_change > 0 else -2)
        va = 'bottom' if percent_change > 0 else 'top'
        ax6_delta.text(angle, y_pos, f'{percent_change:+.1f}%',
                      ha='center', va=va, fontsize=11, fontweight='bold',
                      color=ANGLE_COLORS[angle])

ax6_delta.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
ax6_delta.set_xlabel('Angle of Attack (degrees)', fontsize=12, fontweight='bold', color=BERKELEY_BLUE)
ax6_delta.set_ylabel('Δ Drag (%)', fontsize=12, fontweight='bold', color=BERKELEY_BLUE)
ax6_delta.set_title('Drag Change: Deployed vs Retracted (at max windspeed)',
                   fontsize=13, fontweight='bold', color=BERKELEY_BLUE, pad=10)
ax6_delta.set_xticks([0, 10, 20])
ax6_delta.set_xticklabels(['0°', '10°', '20°'])
ax6_delta.grid(True, alpha=0.2, linestyle='--', color='#AAAAAA', axis='y')
ax6_delta.set_axisbelow(True)
ax6_delta.spines['top'].set_visible(False)
ax6_delta.spines['right'].set_visible(False)
ax6_delta.spines['left'].set_color(BERKELEY_BLUE)
ax6_delta.spines['bottom'].set_color(BERKELEY_BLUE)
ax6_delta.tick_params(colors=BERKELEY_BLUE, which='both', labelsize=10)

plt.tight_layout()
plt.savefig('drag_comprehensive_comparison.png', dpi=300, bbox_inches='tight')
print("  ✓ Saved: drag_comprehensive_comparison.png")
plt.close()

# ============================================================================
# Summary
# ============================================================================

print("\n" + "="*70)
print("Analysis Complete!")
print("="*70)
print("\nGenerated 6 plots:")
print("  1. lift_retracted_all_data.png")
print("  2. lift_deployed_all_data.png")
print("  3. drag_retracted_all_data.png")
print("  4. drag_deployed_all_data.png")
print("  5. lift_comprehensive_comparison.png (with delta)")
print("  6. drag_comprehensive_comparison.png (with delta)")
print()
