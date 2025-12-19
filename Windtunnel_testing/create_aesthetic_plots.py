#!/usr/bin/env python
"""
Aesthetic Wind Tunnel Comprehensive Plots
Modern design with faceted subplots and clean styling
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.patches as mpatches
from scipy import stats
import os

# ============================================================================
# Configuration
# ============================================================================

CALIBRATION_SLOPE = -0.001857
CALIBRATION_INTERCEPT = -1.598511

rpm_to_mph = {
    790: 15.41,
    1188: 25.05,
    1583: 34.00,
    1984: 45.63,
    2382: 57.26,
    2769: 65.76
}

DRAG_LIFT_RATIOS = {0: 0.04, 10: 0.08, 20: 0.15}

# Modern color palette
BERKELEY_BLUE = '#003262'
CALIFORNIA_GOLD = '#FDB515'
RETRACTED_COLOR = '#2E5266'  # Deep blue-grey
DEPLOYED_COLOR = '#9A031E'   # Deep red
ACCENT_COLOR = '#0F8B8D'     # Teal
LIGHT_GREY = '#F0F0F0'
DARK_GREY = '#333333'

# ============================================================================
# Helper Functions
# ============================================================================

def read_data_row(filename):
    df = pd.read_csv(filename, sep='\t', header=21, nrows=1)
    return df

def rpm_to_windspeed(rpm):
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
    return (voltage - intercept) / slope

def decompose_forces(vertical_force, angle_deg):
    alpha_rad = np.radians(angle_deg)
    dl_ratio = DRAG_LIFT_RATIOS.get(angle_deg, 0.08)
    cos_a = np.cos(alpha_rad)
    sin_a = np.sin(alpha_rad)
    denominator = cos_a - dl_ratio * sin_a
    lift = vertical_force / denominator
    drag = dl_ratio * lift
    return lift, drag

def read_configuration_data(config_name, angle_deg, vg_state):
    base_dir = 'wind tunnel data'
    config_dir = os.path.join(base_dir, config_name)

    file_patterns = {
        'retracted__00deg': 'retracted_00_{:02d}.xls',
        'deployed__00deg': 'deployed_{:02d}.xls',
        'retracted_10deg': 'retracted_10_{:02d}.xls',
        'deployed_10deg': 'deployed_10_{:02d}.xls',
        'retracted_20deg': 'retracted_20_{:02d}.xls',
        'deployed__20deg(last3bad)': 'deployed_20_{:02d}.xls'
    }

    file_pattern = file_patterns[config_name]

    if config_name == 'deployed__20deg(last3bad)':
        file_indices = range(1, 7)
    else:
        file_indices = range(0, 7)

    rpms, voltages = [], []
    for i in file_indices:
        filename = os.path.join(config_dir, file_pattern.format(i))
        if os.path.exists(filename):
            df = read_data_row(filename)
            rpms.append(df['Untitled'].values[0])
            voltages.append(df['Untitled 3'].values[0])

    rpms = np.array(rpms)
    voltages = np.array(voltages)
    windspeeds = np.array([rpm_to_windspeed(rpm) for rpm in rpms])
    apparent_weights = voltage_to_force(voltages, CALIBRATION_SLOPE, CALIBRATION_INTERCEPT)

    # Add synthetic baseline for deployed 20deg
    if config_name == 'deployed__20deg(last3bad)' and rpms[0] > 0:
        rpms = np.insert(rpms, 0, 0.0)
        windspeeds = np.insert(windspeeds, 0, 0.0)
        baseline_estimate = apparent_weights[0]
        apparent_weights = np.insert(apparent_weights, 0, baseline_estimate)
        voltages = np.insert(voltages, 0, CALIBRATION_SLOPE * baseline_estimate + CALIBRATION_INTERCEPT)

    baseline_weight = apparent_weights[0]
    vertical_aero_force = baseline_weight - apparent_weights
    lift, drag = decompose_forces(vertical_aero_force, angle_deg)

    return {
        'rpms': rpms, 'voltages': voltages, 'windspeeds': windspeeds,
        'apparent_weights': apparent_weights, 'baseline_weight': baseline_weight,
        'vertical_force': vertical_aero_force, 'lift': lift, 'drag': drag,
        'angle': angle_deg, 'vg_state': vg_state
    }

# ============================================================================
# Load All Data
# ============================================================================

print("Loading data...")
configurations = [
    ('retracted__00deg', 0, 'retracted'),
    ('deployed__00deg', 0, 'deployed'),
    ('retracted_10deg', 10, 'retracted'),
    ('deployed_10deg', 10, 'deployed'),
    ('retracted_20deg', 20, 'retracted'),
    ('deployed__20deg(last3bad)', 20, 'deployed'),
]

all_data = {}
for config_name, angle, vg_state in configurations:
    key = (vg_state, angle)
    all_data[key] = read_configuration_data(config_name, angle, vg_state)
    print(f"  ✓ {vg_state} {angle}°")

# ============================================================================
# AESTHETIC LIFT PLOT - Faceted Design
# ============================================================================

print("\nGenerating aesthetic comprehensive plots...")

fig = plt.figure(figsize=(18, 8))
fig.patch.set_facecolor('white')

# Create 3 subplots + 1 summary
gs = fig.add_gridspec(2, 4, height_ratios=[3.5, 0.6], hspace=0.35, wspace=0.3,
                       left=0.06, right=0.98, top=0.85, bottom=0.08)

# Angle subplots
angles = [0, 10, 20]
angle_axes = []

for i, angle in enumerate(angles):
    ax = fig.add_subplot(gs[0, i])
    angle_axes.append(ax)

    # Set background
    ax.set_facecolor('#FAFAFA')

    # Get data
    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        data_r = all_data[key_r]
        data_d = all_data[key_d]

        ws_r = data_r['windspeeds']
        lift_r = data_r['lift']
        ws_d = data_d['windspeeds']
        lift_d = data_d['lift']

        # Plot curves
        # Retracted
        if len(ws_r) > 2:
            coeffs_r = np.polyfit(ws_r, lift_r, 2)
            poly_r = np.poly1d(coeffs_r)
            ws_smooth_r = np.linspace(0, ws_r.max(), 150)
            lift_smooth_r = poly_r(ws_smooth_r)
            ax.plot(ws_smooth_r, lift_smooth_r, '-', color=RETRACTED_COLOR,
                   linewidth=4, alpha=0.7, label='Retracted', zorder=3)

        ax.scatter(ws_r, lift_r, s=180, alpha=0.9, color=RETRACTED_COLOR,
                  marker='o', edgecolors='white', linewidth=3, zorder=5)

        # Deployed
        if len(ws_d) > 2:
            coeffs_d = np.polyfit(ws_d, lift_d, 2)
            poly_d = np.poly1d(coeffs_d)
            ws_smooth_d = np.linspace(0, ws_d.max(), 150)
            lift_smooth_d = poly_d(ws_smooth_d)
            ax.plot(ws_smooth_d, lift_smooth_d, '-', color=DEPLOYED_COLOR,
                   linewidth=4, alpha=0.7, label='Deployed', zorder=3)

        ax.scatter(ws_d, lift_d, s=180, alpha=0.9, color=DEPLOYED_COLOR,
                  marker='s', edgecolors='white', linewidth=3, zorder=5)

        # Calculate delta
        max_lift_r = np.max(lift_r)
        max_lift_d = np.max(lift_d)
        delta_pct = ((max_lift_d - max_lift_r) / max_lift_r * 100) if max_lift_r != 0 else 0

        # Add delta annotation box
        color = ACCENT_COLOR if delta_pct > 0 else '#E63946'
        box = FancyBboxPatch((0.05, 0.85), 0.35, 0.12, transform=ax.transAxes,
                            boxstyle="round,pad=0.01",
                            facecolor=color, edgecolor='white',
                            alpha=0.9, linewidth=2, zorder=10)
        ax.add_patch(box)

        sign = '+' if delta_pct >= 0 else ''
        ax.text(0.225, 0.91, f'{sign}{delta_pct:.1f}%', transform=ax.transAxes,
               fontsize=18, fontweight='bold', ha='center', va='center',
               color='white', zorder=11)
        ax.text(0.225, 0.86, 'Δ Lift', transform=ax.transAxes,
               fontsize=10, ha='center', va='center',
               color='white', zorder=11, style='italic')

    # Styling
    ax.set_xlabel('Windspeed (mph)', fontsize=13, fontweight='bold', color=DARK_GREY)
    ax.set_ylabel('Lift Force (grams)', fontsize=13, fontweight='bold', color=DARK_GREY)
    ax.set_title(f'{angle}° Angle of Attack', fontsize=16, fontweight='bold',
                color=BERKELEY_BLUE, pad=15)

    if i == 0:
        ax.legend(fontsize=11, loc='lower right', framealpha=0.95,
                 edgecolor=BERKELEY_BLUE, fancybox=True)

    ax.grid(True, alpha=0.3, linestyle='--', color='#CCCCCC', linewidth=0.8)
    ax.set_axisbelow(True)

    for spine in ax.spines.values():
        spine.set_color(BERKELEY_BLUE)
        spine.set_linewidth(2)

    ax.tick_params(colors=DARK_GREY, labelsize=10)

# Summary subplot (4th column)
ax_summary = fig.add_subplot(gs[0, 3])
ax_summary.set_facecolor('#FAFAFA')

# Create summary visualization
y_positions = [2, 1, 0]
colors_summary = [BERKELEY_BLUE, FOUNDERS_ROCK := '#3B7EA1', CALIFORNIA_GOLD]
deltas = []

for idx, angle in enumerate(angles):
    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        max_lift_r = np.max(all_data[key_r]['lift'])
        max_lift_d = np.max(all_data[key_d]['lift'])
        delta_pct = ((max_lift_d - max_lift_r) / max_lift_r * 100) if max_lift_r != 0 else 0
        deltas.append(delta_pct)

        # Horizontal bar
        color = colors_summary[idx]
        ax_summary.barh(y_positions[idx], delta_pct, height=0.6,
                       color=color, alpha=0.8, edgecolor='white', linewidth=2)

        # Value label - position inside the bar to avoid edge overlap
        if delta_pct > 0:
            x_pos = delta_pct - 0.5
            ha = 'right'
        else:
            x_pos = delta_pct + 0.5
            ha = 'left'
        ax_summary.text(x_pos, y_positions[idx], f'{delta_pct:+.1f}%',
                       va='center', ha=ha, fontsize=13, fontweight='bold',
                       color='white')

ax_summary.axvline(x=0, color=DARK_GREY, linewidth=2, zorder=1)
ax_summary.set_yticks(y_positions)
ax_summary.set_yticklabels(['0°', '10°', '20°'], fontsize=12, fontweight='bold')
ax_summary.set_xlabel('Performance Change (%)', fontsize=12, fontweight='bold', color=DARK_GREY)
ax_summary.set_title('Summary', fontsize=16, fontweight='bold', color=BERKELEY_BLUE, pad=15)
ax_summary.grid(True, alpha=0.2, axis='x', linestyle='--', color='#CCCCCC')
ax_summary.set_axisbelow(True)

for spine in ax_summary.spines.values():
    spine.set_color(BERKELEY_BLUE)
    spine.set_linewidth(2)
ax_summary.tick_params(colors=DARK_GREY, labelsize=10)

# Bottom info panel
ax_info = fig.add_subplot(gs[1, :])
ax_info.axis('off')

info_text = (
    "Vortex Generator Performance Analysis  •  "
    "Circles = Retracted  •  Squares = Deployed  •  "
    "Positive Δ = Improvement with VGs deployed"
)
ax_info.text(0.5, 0.5, info_text, transform=ax_info.transAxes,
            fontsize=12, ha='center', va='center',
            style='italic', color=DARK_GREY)

# Main title
fig.suptitle('Lift Force Comprehensive Comparison - All Configurations',
            fontsize=20, fontweight='bold', color=BERKELEY_BLUE, y=0.94)

plt.savefig('figures/comprehensive_comparison/lift_comprehensive_comparison.png',
           dpi=300, bbox_inches='tight', facecolor='white')
print("  ✓ Saved: lift_comprehensive_comparison.png")
plt.close()

# ============================================================================
# AESTHETIC DRAG PLOT - Same Design
# ============================================================================

fig = plt.figure(figsize=(18, 8))
fig.patch.set_facecolor('white')

gs = fig.add_gridspec(2, 4, height_ratios=[3.5, 0.6], hspace=0.35, wspace=0.3,
                       left=0.06, right=0.98, top=0.85, bottom=0.08)

angle_axes = []

for i, angle in enumerate(angles):
    ax = fig.add_subplot(gs[0, i])
    angle_axes.append(ax)
    ax.set_facecolor('#FAFAFA')

    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        data_r = all_data[key_r]
        data_d = all_data[key_d]

        ws_r = data_r['windspeeds']
        drag_r = data_r['drag']
        ws_d = data_d['windspeeds']
        drag_d = data_d['drag']

        # Retracted
        if len(ws_r) > 2:
            coeffs_r = np.polyfit(ws_r, drag_r, 2)
            poly_r = np.poly1d(coeffs_r)
            ws_smooth_r = np.linspace(0, ws_r.max(), 150)
            drag_smooth_r = poly_r(ws_smooth_r)
            ax.plot(ws_smooth_r, drag_smooth_r, '-', color=RETRACTED_COLOR,
                   linewidth=4, alpha=0.7, label='Retracted', zorder=3)

        ax.scatter(ws_r, drag_r, s=180, alpha=0.9, color=RETRACTED_COLOR,
                  marker='o', edgecolors='white', linewidth=3, zorder=5)

        # Deployed
        if len(ws_d) > 2:
            coeffs_d = np.polyfit(ws_d, drag_d, 2)
            poly_d = np.poly1d(coeffs_d)
            ws_smooth_d = np.linspace(0, ws_d.max(), 150)
            drag_smooth_d = poly_d(ws_smooth_d)
            ax.plot(ws_smooth_d, drag_smooth_d, '-', color=DEPLOYED_COLOR,
                   linewidth=4, alpha=0.7, label='Deployed', zorder=3)

        ax.scatter(ws_d, drag_d, s=180, alpha=0.9, color=DEPLOYED_COLOR,
                  marker='s', edgecolors='white', linewidth=3, zorder=5)

        # Delta
        max_drag_r = np.max(drag_r)
        max_drag_d = np.max(drag_d)
        delta_pct = ((max_drag_d - max_drag_r) / max_drag_r * 100) if max_drag_r != 0 else 0

        color = ACCENT_COLOR if delta_pct > 0 else '#E63946'
        box = FancyBboxPatch((0.05, 0.85), 0.35, 0.12, transform=ax.transAxes,
                            boxstyle="round,pad=0.01",
                            facecolor=color, edgecolor='white',
                            alpha=0.9, linewidth=2, zorder=10)
        ax.add_patch(box)

        sign = '+' if delta_pct >= 0 else ''
        ax.text(0.225, 0.91, f'{sign}{delta_pct:.1f}%', transform=ax.transAxes,
               fontsize=18, fontweight='bold', ha='center', va='center',
               color='white', zorder=11)
        ax.text(0.225, 0.86, 'Δ Drag', transform=ax.transAxes,
               fontsize=10, ha='center', va='center',
               color='white', zorder=11, style='italic')

    ax.set_xlabel('Windspeed (mph)', fontsize=13, fontweight='bold', color=DARK_GREY)
    ax.set_ylabel('Drag Force (grams)', fontsize=13, fontweight='bold', color=DARK_GREY)
    ax.set_title(f'{angle}° Angle of Attack', fontsize=16, fontweight='bold',
                color=BERKELEY_BLUE, pad=15)

    if i == 0:
        ax.legend(fontsize=11, loc='lower right', framealpha=0.95,
                 edgecolor=BERKELEY_BLUE, fancybox=True)

    ax.grid(True, alpha=0.3, linestyle='--', color='#CCCCCC', linewidth=0.8)
    ax.set_axisbelow(True)

    for spine in ax.spines.values():
        spine.set_color(BERKELEY_BLUE)
        spine.set_linewidth(2)

    ax.tick_params(colors=DARK_GREY, labelsize=10)

# Summary
ax_summary = fig.add_subplot(gs[0, 3])
ax_summary.set_facecolor('#FAFAFA')

for idx, angle in enumerate(angles):
    key_r = ('retracted', angle)
    key_d = ('deployed', angle)

    if key_r in all_data and key_d in all_data:
        max_drag_r = np.max(all_data[key_r]['drag'])
        max_drag_d = np.max(all_data[key_d]['drag'])
        delta_pct = ((max_drag_d - max_drag_r) / max_drag_r * 100) if max_drag_r != 0 else 0

        color = colors_summary[idx]
        ax_summary.barh(y_positions[idx], delta_pct, height=0.6,
                       color=color, alpha=0.8, edgecolor='white', linewidth=2)

        # Position inside the bar to avoid edge overlap
        if delta_pct > 0:
            x_pos = delta_pct - 0.5
            ha = 'right'
        else:
            x_pos = delta_pct + 0.5
            ha = 'left'
        ax_summary.text(x_pos, y_positions[idx], f'{delta_pct:+.1f}%',
                       va='center', ha=ha, fontsize=13, fontweight='bold',
                       color='white')

ax_summary.axvline(x=0, color=DARK_GREY, linewidth=2, zorder=1)
ax_summary.set_yticks(y_positions)
ax_summary.set_yticklabels(['0°', '10°', '20°'], fontsize=12, fontweight='bold')
ax_summary.set_xlabel('Performance Change (%)', fontsize=12, fontweight='bold', color=DARK_GREY)
ax_summary.set_title('Summary', fontsize=16, fontweight='bold', color=BERKELEY_BLUE, pad=15)
ax_summary.grid(True, alpha=0.2, axis='x', linestyle='--', color='#CCCCCC')
ax_summary.set_axisbelow(True)

for spine in ax_summary.spines.values():
    spine.set_color(BERKELEY_BLUE)
    spine.set_linewidth(2)
ax_summary.tick_params(colors=DARK_GREY, labelsize=10)

# Bottom info
ax_info = fig.add_subplot(gs[1, :])
ax_info.axis('off')
ax_info.text(0.5, 0.5, info_text, transform=ax_info.transAxes,
            fontsize=12, ha='center', va='center',
            style='italic', color=DARK_GREY)

fig.suptitle('Drag Force Comprehensive Comparison - All Configurations',
            fontsize=20, fontweight='bold', color=BERKELEY_BLUE, y=0.94)

plt.savefig('figures/comprehensive_comparison/drag_comprehensive_comparison.png',
           dpi=300, bbox_inches='tight', facecolor='white')
print("  ✓ Saved: drag_comprehensive_comparison.png")
plt.close()

print("\n✅ Aesthetic comprehensive plots complete!")
print("   Location: figures/comprehensive_comparison/")
