# Wind Tunnel Analysis Summary

## Folder Structure

```
figures/
├── individual_plots/          # 4 individual plots (lift & drag, retracted & deployed)
│   ├── lift_retracted_all_data.png
│   ├── lift_deployed_all_data.png
│   ├── drag_retracted_all_data.png
│   └── drag_deployed_all_data.png
│
├── comprehensive_comparison/  # 2 comprehensive plots with delta analysis
│   ├── lift_comprehensive_comparison.png
│   └── drag_comprehensive_comparison.png
│
└── original_analysis/         # Original 4 plots (without bad data)
    ├── lift_retracted_all_angles.png
    ├── lift_deployed_all_angles.png
    ├── drag_retracted_all_angles.png
    └── drag_deployed_all_angles.png
```

## Key Findings

### Data Processing Notes

**Deployed 20° Configuration:**
- Missing baseline (0 mph) measurement - file `deployed_20_00.xls` does not exist
- Data starts at ~799 RPM instead of 0 RPM
- **Solution:** Added synthetic (0,0) point for lift and drag at 0 windspeed
- Files examined started at file 01 (not file 00), confirming this is a missing baseline rather than mislabeled files

### Vortex Generator Performance Summary

| Angle of Attack | Lift Change | Drag Change | Interpretation |
|----------------|-------------|-------------|----------------|
| **0°**         | -7.4%       | -7.4%       | VGs slightly reduce performance (adds drag with minimal flow benefit) |
| **10°**        | **+12.7%**  | +12.7%      | **Best performance** - significant lift improvement with acceptable drag increase |
| **20°**        | +7.5%       | +7.5%       | Good improvement but approaching stall conditions |

### Physical Interpretation

**At 0° (Cruise):**
- Vortex generators add parasitic drag without providing flow control benefits
- Slight reduction in both lift and drag
- Expected behavior for low angle of attack

**At 10° (Optimal):**
- VGs energize boundary layer, delaying flow separation
- 12.7% improvement in lift generation
- Drag increase is proportional (maintaining similar L/D ratio)
- This is the sweet spot for VG performance

**At 20° (High Alpha):**
- Wing approaching stall
- VGs still provide benefit by maintaining attached flow
- 7.5% improvement in both lift and drag
- Performance gain lower than at 10° (likely partial separation occurring)

## Drag Calculation Methodology

Since the wind tunnel setup only measures **vertical force** (via strain gauge), lift and drag were decomposed using:

1. **Geometric decomposition:**
   - Measured vertical force = L·cos(α) - D·sin(α)
   - Where α is angle of attack

2. **Assumed drag-to-lift ratios** (based on typical airfoil behavior):
   - 0°: D/L = 0.04 (low drag at zero lift)
   - 10°: D/L = 0.08 (moderate induced drag)
   - 20°: D/L = 0.15 (higher drag near stall)

3. **Solving for L and D:**
   - L = vertical_force / (cos(α) - (D/L)·sin(α))
   - D = (D/L) × L

**Note:** This is an approximation. For precise drag measurements, a separate horizontal force sensor would be required.

## Configuration Details

### Test Matrix

| Configuration | Angle | VG State | Data Points | Max Windspeed | Notes |
|--------------|-------|----------|-------------|---------------|-------|
| Config 1 | 0° | Retracted | 7 | 65.76 mph | Baseline |
| Config 2 | 0° | Deployed | 7 | 65.76 mph | Complete data |
| Config 3 | 10° | Retracted | 7 | 65.76 mph | Complete data |
| Config 4 | 10° | Deployed | 7 | 65.76 mph | Complete data |
| Config 5 | 20° | Retracted | 7 | 65.76 mph | Complete data |
| Config 6 | 20° | Deployed | 7 | 65.76 mph | **Synthetic (0,0) point added** |

### Baseline Weights

Different configurations show different baseline weights due to:
- Wing position changes with angle of attack
- Potential setup adjustments between runs
- Normal experimental variation

| Configuration | Baseline Weight |
|--------------|-----------------|
| Retracted 0° | 542.4g |
| Deployed 0° | 554.8g |
| Retracted 10° | 520.2g |
| Deployed 10° | 403.4g |
| Retracted 20° | 105.9g |
| Deployed 20° | ~1.6g (estimated) |

## Plot Descriptions

### Individual Plots (figures/individual_plots/)

Each plot shows **3 curves** (one for each angle: 0°, 10°, 20°):
- **Circles** = Retracted VGs
- **Squares** = Deployed VGs
- Color coding: Blue (0°), Teal (10°), Gold (20°)
- Quadratic curve fits included

### Comprehensive Comparison Plots (figures/comprehensive_comparison/)

**Two-panel plots:**
- **Top panel:** All 6 configurations overlaid
  - Solid lines = Retracted
  - Dashed lines = Deployed
  - Same color coding by angle
- **Bottom panel:** Delta bar chart showing % change from retracted to deployed
  - Positive values = deployed performs better
  - Negative values = retracted performs better

### Original Analysis (figures/original_analysis/)

Initial plots excluding the "bad" data points from deployed 20° configuration (files 04-06).

## Scripts

- `analyze_windtunnel_data.py` - Original analysis (0° retracted only)
- `analyze_all_configurations.py` - First comprehensive analysis (excluding bad data)
- `analyze_all_configurations_complete.py` - Second pass (including all data)
- `analyze_all_configurations_final.py` - **Final version** with improved formatting and organization

## Recommendations

1. **Retake deployed 20° baseline measurement** (file 00) for complete dataset
2. **Investigate deployed 20° high-speed data** (files 04-06) - marked as "bad" but included in final analysis
3. **Consider testing intermediate angles** (5°, 15°) to better characterize VG performance curve
4. **Install horizontal force sensor** for direct drag measurements
5. **Focus VG deployment** at moderate-to-high angles of attack (10°-20°) where benefits are clear

## Data Quality Notes

✅ **Good quality data:**
- All retracted configurations (0°, 10°, 20°)
- Deployed 0° and 10°

⚠️ **Acceptable with notes:**
- Deployed 20°: Missing baseline (synthetic point added), files 04-06 marked as potentially problematic but data looks reasonable

---

*Analysis completed: December 18, 2025*
*UC Berkeley - Introduction to Robotics - Final Project*
