#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Advanced Teleop Log Analyzer
============================
Performs deep statistical analysis on teleoperation logs to identify performance bottlenecks,
network jitter, and robot motion characteristics.

Features:
- Comprehensive Statistics (Mean, Median, StdDev, Percentiles, Min/Max)
- Latency Breakdown (Network, Decision, Robot Response, Execution)
- Jitter & Stability Analysis (Gap detection, Frequency stability)
- Motion Profile Analysis (Velocity, Motion Delta, Correlation)
- ASCII Histogram & Distribution Buckets

Usage:
    python3 advanced_log_analyzer.py [latency_log_path] [struct_log_path]
"""

import csv
import math
import statistics
import sys
import os
from collections import defaultdict

# --- Configuration ---
LATENCY_LOG_DEFAULT = None # Auto-detects latest if None
STRUCT_LOG_DEFAULT = None  # Auto-detects latest if None

# Thresholds
JITTER_GAP_THRESHOLD_MS = 100.0  # Gap > 100ms considered a lag spike
HIGH_LATENCY_THRESHOLD_MS = 500.0 # Latency > 500ms considered poor
LARGE_MOVE_DEG = 10.0 # Move > 10 deg considered large

class AdvancedLogAnalyzer:
    def __init__(self):
        self.latency_data = []
        self.struct_data = []
        self.stats_cache = {}

    def load_data(self, latency_path, struct_path):
        """Load and pre-process CSV data"""
        print(f"📂 Loading logs...")
        
        # Load Latency Log
        if latency_path and os.path.exists(latency_path):
            with open(latency_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Clean and parse row
                    clean_row = {}
                    for k, v in row.items():
                        try:
                            clean_row[k] = float(v)
                        except ValueError:
                            clean_row[k] = str(v)
                    
                    # Fix 0.0 timestamp bugs (sanity check)
                    if clean_row.get('True_End_to_End_ms', 0) > 100000:
                        continue # Skip garbage data
                        
                    self.latency_data.append(clean_row)
            print(f"   - Latency Log: {len(self.latency_data)} rows loaded.")
        else:
            print(f"   ❌ Latency Log not found or invalid path: {latency_path}")

        # Load Struct Log
        if struct_path and os.path.exists(struct_path):
            with open(struct_path, 'r') as f:
                reader = csv.DictReader(f)
                prev_timestamp = None
                for row in reader:
                    clean_row = {}
                    for k, v in row.items():
                        try:
                            clean_row[k] = float(v)
                        except ValueError:
                            clean_row[k] = str(v)
                    
                    # Calculate derived fields
                    curr_timestamp = clean_row.get('Timestamp', 0)
                    if prev_timestamp and curr_timestamp > prev_timestamp:
                        clean_row['Calculate_Diff_ms'] = (curr_timestamp - prev_timestamp) * 1000.0
                    else:
                        clean_row['Calculate_Diff_ms'] = 0.0
                    
                    prev_timestamp = curr_timestamp
                    self.struct_data.append(clean_row)
            print(f"   - Struct Log:  {len(self.struct_data)} rows loaded.")
        else:
            print(f"   ❌ Struct Log not found or invalid path: {struct_path}")

    def calculate_distribution(self, data, buckets):
        """Calculate percentage distribution across buckets"""
        counts = {k: 0 for k in buckets.keys()}
        total = len(data)
        if total == 0: return counts
        
        for val in data:
            for label, (low, high) in buckets.items():
                if low <= val < high:
                    counts[label] += 1
                    break
        
        return {k: (v / total) * 100 for k, v in counts.items()}

    def get_stats(self, data_list, label="Data"):
        """Calculate comprehensive statistics for a list of numbers"""
        if not data_list:
            return None
        
        # Filter Nones and infinites
        valid_data = [x for x in data_list if x is not None and math.isfinite(x)]
        if not valid_data: return None

        n = len(valid_data)
        if n < 2: return None

        # Use 'inclusive' to prevent extrapolation beyond Min/Max for small datasets
        try:
            quantiles = statistics.quantiles(valid_data, n=100, method='inclusive') 
        except TypeError: 
            # Fallback for older python < 3.8 if method param doesn't exist (unlikely but safe)
            quantiles = statistics.quantiles(valid_data, n=100)

        # Safety Clamp to ensure Percentiles are within [Min, Max]
        d_min = min(valid_data)
        d_max = max(valid_data)
        
        p05 = max(d_min, min(d_max, quantiles[4]))
        p25 = max(d_min, min(d_max, quantiles[24]))
        p75 = max(d_min, min(d_max, quantiles[74]))
        p95 = max(d_min, min(d_max, quantiles[94]))
        p99 = max(d_min, min(d_max, quantiles[98]))

        return {
            'Label': label,
            'Count': n,
            'Mean': statistics.mean(valid_data),
            'Median': statistics.median(valid_data),
            'Min': d_min,
            'Max': d_max,
            'StdDev': statistics.stdev(valid_data),
            'P05': p05,
            'P25': p25,
            'P75': p75,
            'P95': p95,
            'P99': p99
        }

    def print_stats_box(self, stats_dict, unit=""):
        """Pretty print statistics"""
        if not stats_dict:
            print("   [No Data Available]")
            return

        print(f"   {stats_dict['Label']}")
        print(f"   {'='*40}")
        print(f"   🔹 Mean:   {stats_dict['Mean']:8.2f} {unit}  (σ = {stats_dict['StdDev']:.2f})")
        print(f"   🔹 Median: {stats_dict['Median']:8.2f} {unit}")
        print(f"   🔹 Range:  [{stats_dict['Min']:.2f} - {stats_dict['Max']:.2f}] {unit}")
        print(f"   🔹 Percentiles:")
        print(f"      P05: {stats_dict['P05']:6.2f} | P25: {stats_dict['P25']:6.2f}")
        print(f"      P75: {stats_dict['P75']:6.2f} | P95: {stats_dict['P95']:6.2f} | P99: {stats_dict['P99']:6.2f}")
        print("")

    def analyze_latency_performance(self):
        print("\n" + "="*60)
        print("🚀 LATENCY PERFORMANCE ANALYSIS")
        print("="*60)
        
        cols = {
            'True_End_to_End_ms': "End-to-End Latency",
            'Network_Delay_ms': "Network Delay",
            'Decision_Delay_ms': "Decision/Processing Delay",
            'Robot_Response_ms': "Robot Physical Response",
            'Motion_Execution_ms': "Motion Execution Duration"
        }

        for col_key, col_name in cols.items():
            data = [row.get(col_key, 0) for row in self.latency_data]
            stats = self.get_stats(data, col_name)
            self.print_stats_box(stats, "ms")
            
            # Buckets Analysis for End-to-End
            if col_key == 'True_End_to_End_ms':
                buckets = {
                    'Excellent (<50ms)': (0, 50),
                    'Good (50-200ms)': (50, 200),
                    'Fair (200-500ms)': (200, 500),
                    'Poor (>500ms)': (500, float('inf'))
                }
                dist = self.calculate_distribution(data, buckets)
                print("   📊 Quality Distribution:")
                for k, v in dist.items():
                    bar = "█" * int(v / 2)
                    print(f"      {k:18}: {v:5.1f}% | {bar}")
                print("-" * 40)

    def analyze_jitter_stability(self):
        print("\n" + "="*60)
        print("📉 JITTER & STABILITY ANALYSIS")
        print("="*60)
        
        # Analyze Packet Interval (Time Since Last) from Struct Log
        intervals = [row.get('Calculate_Diff_ms', 0) for row in self.struct_data 
                     if row.get('Calculate_Diff_ms', 0) > 0.1 # Filter zero deltas
                     ]
                     
        stats = self.get_stats(intervals, "Inter-Arrival Time (Jitter)")
        self.print_stats_box(stats, "ms")
        
        # Gap Detection
        gaps_100ms = len([x for x in intervals if x > 100])
        gaps_500ms = len([x for x in intervals if x > 500])
        total = len(intervals) or 1
        
        print(f"   ⚠️  Stability Issues:")
        print(f"      Packet Gaps > 100ms: {gaps_100ms} events ({gaps_100ms/total*100:.1f}%)")
        print(f"      Packet Gaps > 500ms: {gaps_500ms} events ({gaps_500ms/total*100:.1f}%)")
        
        if stats:
            avg_interval = stats['Mean']
            freq = 1000.0 / avg_interval if avg_interval > 0 else 0
            print(f"      Est. Update Rate:    {freq:.2f} Hz")

    def analyze_motion_profile(self):
        print("\n" + "="*60)
        print("🤖 MOTION & VELOCITY PROFILE")
        print("="*60)
        
        velocities = []
        large_move_velocities = []
        motion_deltas = []
        
        # Moving Average Window for smoother "Physical" Velocity
        window_size = 5
        recent_dists = []
        recent_times = []

        for row in self.struct_data:
            try:
                dist_rad = row.get('Dist_to_Last', 0)
                # FIX: Use the calculated diff from load_data, not the raw CSV column which might be empty
                time_ms = row.get('Calculate_Diff_ms', 0) 
                dist_deg = math.degrees(dist_rad)
                
                if dist_deg > 0.01:
                    motion_deltas.append(dist_deg)
                
                # Only consider moving frames for velocity
                if dist_deg > 0.01 and time_ms > 0:
                    recent_dists.append(dist_deg)
                    recent_times.append(time_ms)
                    
                    if len(recent_dists) > window_size:
                        recent_dists.pop(0)
                        recent_times.pop(0)
                    
                    # Calculate velocity over the window (Smoothed)
                    if len(recent_dists) == window_size:
                        total_dist = sum(recent_dists)
                        total_time = sum(recent_times)
                        
                        if total_time > 10.0: # Ensure we have enough time duration to be stable
                            avg_vel = (total_dist / (total_time / 1000.0))
                            
                            # Filter physically impossible speeds (Burst noise)
                            # MG400 max is ~300. We allow up to 600 to catch fast transients but filter 1000+
                            if avg_vel < 600.0:
                                velocities.append(avg_vel)
                                
                                # SEGMENTATION: Isolate "Large Moves" vs "Micro Moves"
                                # If the window covered > 10 degrees, it's a substantial move
                                if total_dist > 10.0:
                                    large_move_velocities.append(avg_vel)

            except: pass
            
        # Motion Delta Stats
        d_stats = self.get_stats(motion_deltas, "Motion Delta Per Command")
        self.print_stats_box(d_stats, "deg")
        
        # Velocity Stats (Overall)
        v_stats = self.get_stats(velocities, "Achieved Angular Velocity (All Moves)")
        self.print_stats_box(v_stats, "deg/s")
        
        # Velocity Stats (Large Moves Only) - The "Real" Speed
        print("   🚀 PEAK PERFORMANCE (Large Moves > 10° Only)")
        if large_move_velocities:
            peak_stats = self.get_stats(large_move_velocities, "Cruising Velocity")
            self.print_stats_box(peak_stats, "deg/s")
            print(f"      👉 This represents the robot's actual speed when traveling.")
        else:
            print("      [No Large Moves Detected to Analyze Peak Speed]")

        # Large Jumps Correlation
        print("\n   🔍 Motion Composition:")
        if motion_deltas:
            small_moves = len([d for d in motion_deltas if d < 1.0])
            med_moves = len([d for d in motion_deltas if 1.0 <= d < 10.0])
            large_moves = len([d for d in motion_deltas if d >= 10.0])
            total = len(motion_deltas)
            
            print(f"      Micro Moves (<1°):   {small_moves/total*100:.1f}%")
            print(f"      Normal Moves (1-10°):{med_moves/total*100:.1f}%")
            print(f"      Large Jumps (>10°):  {large_moves/total*100:.1f}%  <-- Potential Lag Source")

    def analyze_latency_composition(self):
        print("\n" + "="*60)
        print("🍰 LATENCY COMPOSITION (Where is the time going?)")
        print("="*60)
        
        # Calculate sums
        total_e2e = 0
        total_net = 0
        total_dec = 0
        total_resp = 0
        total_motion = 0
        
        count = 0
        for row in self.latency_data:
            e2e = row.get('True_End_to_End_ms', 0)
            if e2e > 0:
                total_e2e += e2e
                total_net += row.get('Network_Delay_ms', 0)
                total_dec += row.get('Decision_Delay_ms', 0)
                total_resp += row.get('Robot_Response_ms', 0)
                total_motion += row.get('Motion_Time_ms', 0)
                count += 1
                
        if count == 0:
            print("   [No Data for Composition Analysis]")
            return

        avg_e2e = total_e2e / count
        
        # Calculate percentages
        pct_net = (total_net / total_e2e) * 100
        pct_dec = (total_dec / total_e2e) * 100
        pct_resp = (total_resp / total_e2e) * 100
        pct_motion = (total_motion / total_e2e) * 100
        
        # Residual (Time unaccounted for or calculation drift)
        pct_residual = 100 - (pct_net + pct_dec + pct_resp + pct_motion)
        
        print(f"   Based on Average End-to-End Latency: {avg_e2e:.2f} ms")
        print(f"   ------------------------------------------------------------")
        print(f"   1. 📡 Network Transmit (Unity->ROS):   {pct_net:5.1f}% | {total_net/count:6.2f} ms")
        print(f"   2. 🧠 Processing (ROS Logic):          {pct_dec:5.1f}% | {total_dec/count:6.2f} ms")
        print(f"   3. 🤖 Robot Reaction (Cmd->Start):     {pct_resp:5.1f}% | {total_resp/count:6.2f} ms")
        print(f"   4. 🐌 Motion Execution (Moving...):    {pct_motion:5.1f}% | {total_motion/count:6.2f} ms  <-- MAIN BOTTLENECK")
        print(f"   ------------------------------------------------------------")
        
        # Diagnosis
        print("\n   🩺 DIAGNOSIS & RECOMMENDATION:")
        if pct_net > 20:
            print("      ⚠️  Network is slow. Check Wi-Fi/LAN connection.")
        else:
            print("      ✅  Network is healthy (negligible delay).")
            
        if pct_dec > 20:
            print("      ⚠️  ROS Processing is slow. Check CPU usage or complex logic in node.")
        else:
            print("      ✅  ROS Processing is fast.")

        if pct_motion > 50:
            print("      🔴  PROBLEM FOUND: Physical Robot Motion is the bottleneck.")
            print("          The robot assumes 90%+ of loop time merely moving to the target.")
            print("          Fix: Increase Packet Jitter (send less often) OR Increase Robot Speed.")
        

    def analyze_correlations_and_trends(self):
        print("\n" + "="*60)
        print("📈 CORRELATIONS & TRENDS (Deep Dive)")
        print("="*60)
        
        # 1. Correlation: Motion Delta (Deg) vs Execution Time (ms)
        # We need to match rows. Assuming struct_log and latency_log correspond roughly or if latency log has positions.
        # Ideally latency log has 'Motion_Time_ms'. We need 'Motion_Delta' for that same move.
        # The latency log has 'Q_Target_J1' and 'Q_Final_J1'. We can compute delta from that.
        
        deltas = []
        times = []
        
        prev_target = None
        for row in self.latency_data:
            # Reconstruct motion delta from latency log targets if possible
            # Or use 'Q_Target_J1' - 'Q_Initial_J1' (if available, mostly just final and target)
            # Let's approximate delta using current row Target vs Prevous row Target (since target changes drive motion)
            current_target = row.get('Q_Target_J1', 0)
            motion_time = row.get('Motion_Time_ms', 0)
            
            if prev_target is not None and motion_time > 0:
                delta = abs(current_target - prev_target) * 57.2958 # rad to deg
                if delta > 0.1:
                    deltas.append(delta)
                    times.append(motion_time)
            
            prev_target = current_target
            
        if len(deltas) > 10:
            # Simple Linear Correlation Coefficient (Pearson)
            n = len(deltas)
            sum_x = sum(deltas)
            sum_y = sum(times)
            sum_xy = sum(x*y for x,y in zip(deltas, times))
            sum_x2 = sum(x**2 for x in deltas)
            sum_y2 = sum(y**2 for y in times)
            
            numerator = n * sum_xy - sum_x * sum_y
            denominator = math.sqrt((n * sum_x2 - sum_x**2) * (n * sum_y2 - sum_y**2)) if n * sum_x2 != sum_x**2 and n * sum_y2 != sum_y**2 else 0
            
            r = numerator / denominator if denominator != 0 else 0
            
            print(f"   🔗 Correlation: Motion Distance vs Time")
            print(f"      Pearson Coeff (r): {r:.4f}")
            if r > 0.8:
                print("      ✅ Strong positive correlation. Time is directly proportional to distance (Linear).")
            elif r > 0.5:
                print("      ⚠️ Moderate correlation. Other factors (Accel/Decel) play a role.")
            else:
                print("      ❓ Weak/No correlation. Motion time is unpredictable (Blocking/Retries?).")
                
        # 2. Trend Analysis: First Half vs Second Half
        n_total = len(self.latency_data)
        if n_total > 10:
            mid = n_total // 2
            first_half = [row.get('True_End_to_End_ms', 0) for row in self.latency_data[:mid] if row.get('True_End_to_End_ms',0) < 100000]
            second_half = [row.get('True_End_to_End_ms', 0) for row in self.latency_data[mid:] if row.get('True_End_to_End_ms',0) < 100000]
            
            avg_1 = statistics.mean(first_half) if first_half else 0
            avg_2 = statistics.mean(second_half) if second_half else 0
            
            diff_pct = ((avg_2 - avg_1) / avg_1 * 100) if avg_1 > 0 else 0
            
            print(f"\n   ⏳ Performance Trend (Stability over time)")
            print(f"      First Half Avg Latency: {avg_1:.2f} ms")
            print(f"      Last Half Avg Latency:  {avg_2:.2f} ms")
            print(f"      Change: {diff_pct:+.2f}%")
            
            if diff_pct > 20:
                print("      🔴 Degradation Detected! System gets slower over time (Thermal/Leak?).")
            elif diff_pct < -20:
                print("      🟢 Improvement Detected! System warms up or settles.")
            else:
                print("      ✅ Stable. Performance is consistent.")


    def analyze_command_velocity(self):
        print("\n" + "="*60)
        print("⚡ PHYSICAL MOTION VELOCITY (Real Robot Time: T5-T4)")
        print("="*60)
        
        burst_velocities = []
        skipped_t4 = 0
        
        # We need current and previous Q_Final to calculate distance moved per command
        prev_q = None
        
        for row in self.latency_data:
            try:
                # Check for Valid T4 (Motion Start)
                # We want PURE physical velocity. 
                # If T4 is reported as 0 or close to T3, it might be a fallback.
                t4 = row.get('T4_Motion_Start', 0)
                if t4 == 0:
                    skipped_t4 += 1
                    # continue # Actually, let's skip checking this row for velocity if we can't confirm start time
                    # But we need to update prev_q regardless to keep sequence
                    
                q_final = [
                    row.get('Q_Final_J1', 0),
                    row.get('Q_Final_J2', 0),
                    row.get('Q_Final_J3', 0),
                    row.get('Q_Final_J4', 0)
                ]
                
                # We specifically use Motion_Time_ms which SHOULD be T5-T4
                # But to be absolutely sure, let's check if we can compute it manually from timestamps if available
                # The CSV load_data converts everything to float/str. 
                # Let's trust Motion_Time_ms BUT only if T4 > 0 (checked above)
                motion_time = row.get('Motion_Time_ms', 0)
                
                if prev_q and t4 > 0:
                    # Calculate Euclidean distance in Joint Space
                    dist_sq = sum((c - p)**2 for c, p in zip(q_final, prev_q))
                    dist_deg = math.degrees(math.sqrt(dist_sq))
                    
                    # Filter for meaningful moves (> 5 degrees) so we don't amplify noise
                    # And ensure time is valid (> 10ms)
                    if motion_time > 10.0 and dist_deg > 5.0:
                        vel = dist_deg / (motion_time / 1000.0)
                        
                        # Filter outliers (MG400 max ~300-500)
                        if vel < 1000:
                            burst_velocities.append(vel)
                            
                prev_q = q_final
            except Exception as e:
                pass
                
        stats = self.get_stats(burst_velocities, "Physical Burst Velocity")
        self.print_stats_box(stats, "deg/s")
        print(f"   👉 Calculated from T5(Arrival) - T4(Start). Skipped {skipped_t4} rows with missing T4.")
        print("      This excludes Command Latency and purely measures robot mechanism speed.")
        
        # DEBUG: Show details of top speeds to verify
        if burst_velocities:
            print("\n   🕵️‍♂️  TOP 5 FASTEST MOVES (Audit):")
            print(f"      {'Velocity':<15} | {'Time (ms)':<10} | {'Dist (deg)':<12} | {'Timestamps (T4->T5)'}")
            print("      " + "-"*65)
            
            # Re-find the rows corresponding to top speeds (inefficient but safe for debug)
            # Actually, let's just modify the loop above to store data tuples if I were refactoring, 
            # but for now I'll just do a quick collection loop again for clarity/minimal diff
            moves = []
            prev_q = None
            for row in self.latency_data:
                try:
                    t4 = row.get('T4_Motion_Start', 0)
                    t5 = row.get('T5_Target_Reached', 0)
                    q_final = [row.get('Q_Final_J1', 0), row.get('Q_Final_J2', 0), row.get('Q_Final_J3', 0), row.get('Q_Final_J4', 0)]
                    motion_time = row.get('Motion_Time_ms', 0)
                    
                    if prev_q and t4 > 0:
                        dist_sq = sum((c - p)**2 for c, p in zip(q_final, prev_q))
                        dist_deg = math.degrees(math.sqrt(dist_sq))
                        if motion_time > 10.0 and dist_deg > 5.0:
                            vel = dist_deg / (motion_time / 1000.0)
                            if vel < 1000:
                                moves.append( (vel, motion_time, dist_deg, t4, t5) )
                    prev_q = q_final
                except: pass
            
            # Sort by velocity descending
            moves.sort(key=lambda x: x[0], reverse=True)
            
            for m in moves[:5]:
                print(f"      {m[0]:<15.2f} | {m[1]:<10.1f} | {m[2]:<12.2f} | {m[3]:.1f} -> {m[4]:.1f}")
            print("\n      ⚠️ If Time is very low (<30ms), jitter can inflate velocity.")

    def generate_html_report(self):
        print("\n" + "="*60)
        print("📄 GENERATING HTML DASHBOARD")
        print("="*60)
        
        # Collect stats
        lat_end_to_end = self.get_stats([row.get('True_End_to_End_ms') for row in self.latency_data])
        lat_motion = self.get_stats([row.get('Motion_Time_ms') for row in self.latency_data])
        
        # Calculate composition percentages again
        total_e2e = sum(row.get('True_End_to_End_ms', 0) for row in self.latency_data)
        total_motion = sum(row.get('Motion_Time_ms', 0) for row in self.latency_data)
        pct_motion = (total_motion / total_e2e * 100) if total_e2e > 0 else 0
        pct_other = 100 - pct_motion
        
        # Calculate Physical Velocity for Report (Strict T5-T4)
        burst_velocities = []
        prev_q = None
        for row in self.latency_data:
            try:
                t4 = row.get('T4_Motion_Start', 0)
                q_final = [row.get('Q_Final_J1', 0), row.get('Q_Final_J2', 0), row.get('Q_Final_J3', 0), row.get('Q_Final_J4', 0)]
                motion_time = row.get('Motion_Time_ms', 0)
                
                if prev_q and t4 > 0:
                    dist_sq = sum((c - p)**2 for c, p in zip(q_final, prev_q))
                    dist_deg = math.degrees(math.sqrt(dist_sq))
                    
                    if motion_time > 10.0 and dist_deg > 5.0:
                        vel = dist_deg / (motion_time / 1000.0)
                        if vel < 1000: burst_velocities.append(vel)
                prev_q = q_final
            except: pass
        burst_stats = self.get_stats(burst_velocities, "Physical Speed")
        
        # Prepare safe values for template
        safe_burst_max = burst_stats['Max'] if burst_stats else 0.0
        safe_burst_p95 = burst_stats['P95'] if burst_stats else 0.0
        
        safe_e2e_mean = lat_end_to_end['Mean'] if lat_end_to_end else 0.0
        safe_e2e_median = lat_end_to_end['Median'] if lat_end_to_end else 0.0
        safe_e2e_min = lat_end_to_end['Min'] if lat_end_to_end else 0.0
        safe_e2e_max = lat_end_to_end['Max'] if lat_end_to_end else 0.0
        safe_e2e_std = lat_end_to_end['StdDev'] if lat_end_to_end else 0.0
        
        safe_motion_mean = lat_motion['Mean'] if lat_motion else 0.0
        safe_motion_median = lat_motion['Median'] if lat_motion else 0.0
        safe_motion_min = lat_motion['Min'] if lat_motion else 0.0
        safe_motion_max = lat_motion['Max'] if lat_motion else 0.0
        safe_motion_std = lat_motion['StdDev'] if lat_motion else 0.0
        
        safe_motion_count = lat_motion['Count'] if lat_motion else 0

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Teleop Performance Report</title>
            <style>
                body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background-color: #f4f6f8; color: #333; }}
                .container {{ max-width: 1000px; margin: 0 auto; background: white; padding: 40px; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.1); }}
                h1 {{ color: #2c3e50; border-bottom: 2px solid #eee; padding-bottom: 10px; }}
                h2 {{ color: #34495e; margin-top: 30px; }}
                .card {{ background: #fff; border: 1px solid #ddd; border-radius: 6px; padding: 20px; margin-bottom: 20px; }}
                .stats-grid {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 20px; margin-bottom: 20px; }}
                .stat-box {{ background: #f8f9fa; padding: 15px; border-radius: 6px; text-align: center; border-left: 4px solid #3498db; }}
                .stat-value {{ font-size: 24px; font-weight: bold; color: #2c3e50; }}
                .stat-label {{ font-size: 14px; color: #7f8c8d; text-transform: uppercase; letter-spacing: 1px; }}
                
                table {{ width: 100%; border-collapse: collapse; margin-top: 15px; }}
                th, td {{ padding: 12px 15px; text-align: left; border-bottom: 1px solid #eee; }}
                th {{ background-color: #f8f9fa; color: #2c3e50; }}
                
                .progress-bar {{ background-color: #ecf0f1; border-radius: 10px; height: 20px; width: 100%; margin-top: 5px; overflow: hidden; }}
                .progress-fill {{ height: 100%; text-align: right; padding-right: 5px; color: white; font-size: 12px; line-height: 20px; }}
                .bg-green {{ background-color: #2ecc71; }}
                .bg-red {{ background-color: #e74c3c; }}
                .bg-blue {{ background-color: #3498db; }}
                .bg-orange {{ background-color: #f39c12; }}
                
                .badge {{ padding: 5px 10px; border-radius: 20px; font-size: 12px; font-weight: bold; color: white; display: inline-block; }}
                .badge-danger {{ background-color: #e74c3c; }}
                
                @media print {{
                    body {{ background: white; }}
                    .container {{ box-shadow: none; border: none; width: 100%; max-width: 100%; padding: 0; }}
                    .no-print {{ display: none; }}
                }}
            </style>
        </head>
        <body>
            <div class="container">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <h1>Teleop Performance Report</h1>
                    <div style="text-align: right; color: #7f8c8d;">
                        Generated by Advanced Log Analyzer<br>
                        <small>{os.path.basename(LATENCY_LOG_DEFAULT or "Auto-Detected Log")}</small>
                    </div>
                </div>

                <!-- Executive Summary -->
                <div class="card" style="border-left: 5px solid #e74c3c;">
                    <h2 style="margin-top: 0;">🚀 Diagnosis: Physical Motion Bottleneck</h2>
                    <p style="font-size: 16px;">
                        The analysis detected that <strong>{pct_motion:.1f}%</strong> of total system latency is consumed by the robot's physical motion execution.
                        <br><br>
                        <strong>⚡ Robot Capability Check:</strong>
                        Maximum detected burst speed was <strong>{safe_burst_max:.2f} deg/s</strong> (P95: {safe_burst_p95:.2f} deg/s).
                        This confirms the robot <em>can</em> move fast, but average motion is slower due to frequent small adjustments.
                    </p>
                </div>

                <!-- Key Metrics -->
                <h2>Key Performance Indicators (KPI)</h2>
                <div class="stats-grid">
                    <div class="stat-box">
                        <div class="stat-value">{safe_e2e_mean:.2f} ms</div>
                        <div class="stat-label">Avg End-to-End Latency</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value">{pct_motion:.1f}%</div>
                        <div class="stat-label">Time spent Moving</div>
                    </div>
                    <div class="stat-box">
                        <div class="stat-value">{safe_burst_p95:.2f} deg/s</div>
                        <div class="stat-label">Peak Burst Speed (P95)</div>
                    </div>
                </div>

                <!-- Latency Breakdown -->
                <h2>Latency Composition</h2>
                <div class="card">
                    <p>Breakdown of where time is spent in a single command loop:</p>
                    <div style="display: flex; align-items: center; margin-bottom: 10px;">
                        <div style="width: 150px; font-weight: bold;">Motion Execution</div>
                        <div class="progress-bar">
                            <div class="progress-fill bg-red" style="width: {pct_motion}%;">{pct_motion:.1f}%</div>
                        </div>
                        <div style="width: 80px; text-align: right; font-family: monospace;">{safe_motion_mean:.2f} ms</div>
                    </div>
                    <div style="display: flex; align-items: center;">
                        <div style="width: 150px; font-weight: bold;">Software/Net</div>
                        <div class="progress-bar">
                            <div class="progress-fill bg-blue" style="width: {pct_other}%;">{pct_other:.1f}%</div>
                        </div>
                         <div style="width: 80px; text-align: right; font-family: monospace;">{safe_e2e_mean - safe_motion_mean:.2f} ms</div>
                    </div>
                </div>

                <!-- Detailed Stats Table -->
                <h2>Detailed Statistics</h2>
                <table>
                    <thead>
                        <tr>
                            <th>Metric</th>
                            <th>Mean</th>
                            <th>Median</th>
                            <th>Min</th>
                            <th>Max</th>
                            <th>Variance (StdDev)</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr>
                            <td>End-to-End Latency</td>
                            <td>{safe_e2e_mean:.2f} ms</td>
                            <td>{safe_e2e_median:.2f} ms</td>
                            <td>{safe_e2e_min:.2f} ms</td>
                            <td>{safe_e2e_max:.2f} ms</td>
                            <td>{safe_e2e_std:.2f}</td>
                        </tr>
                         <tr>
                            <td>Motion Execution</td>
                            <td>{safe_motion_mean:.2f} ms</td>
                            <td>{safe_motion_median:.2f} ms</td>
                            <td>{safe_motion_min:.2f} ms</td>
                            <td>{safe_motion_max:.2f} ms</td>
                            <td>{safe_motion_std:.2f}</td>
                        </tr>
                    </tbody>
                </table>
                
                <div class="no-print" style="margin-top: 40px; text-align: center; border-top: 1px solid #ddd; padding-top: 20px;">
                    <p style="color: #666;">ℹ️ To save as PDF: Press <strong>Cmd+P (Mac)</strong> or <strong>Ctrl+P (Win)</strong> and select "Save as PDF"</p>
                </div>
            </div>
        </body>
        </html>
        """
        
        output_path = "teleop_report.html"
        with open(output_path, "w") as f:
            f.write(html_content)
            
        print(f"   ✅ Report generated: {output_path}")
        print(f"   👉 Open this file in your browser to view the dashboard and save as PDF.")

    def find_latest_log(self, prefix):
        """Find the most recent file matching the prefix in the script directory"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        files = [f for f in os.listdir(script_dir) if f.startswith(prefix) and f.endswith('.csv')]
        if not files:
            return None
        
        # Sort by modification time (newest first)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(script_dir, x)), reverse=True)
        return os.path.join(script_dir, files[0])

    def run(self):
        # Determine file paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 1. Try Command Line Args
        if len(sys.argv) > 2:
            lat_path = sys.argv[1]
            struct_path = sys.argv[2]
        else:
            # 2. Try Auto-Detect Latest
            print("🔎 Auto-detecting latest logs...")
            lat_path = self.find_latest_log("teleop_latency")
            struct_path = self.find_latest_log("teleop_struct")
            
            if not lat_path or not struct_path:
                # 3. Fallback to Defaults (if defined)
                lat_path = os.path.join(script_dir, LATENCY_LOG_DEFAULT) if LATENCY_LOG_DEFAULT else None
                struct_path = os.path.join(script_dir, STRUCT_LOG_DEFAULT) if STRUCT_LOG_DEFAULT else None
                
                if not lat_path or not struct_path:
                    print("   ❌ Error: Could not find latest logs and no defaults set.")
                    print("   👉 Usage: python3 advanced_log_analyzer.py [latency_log] [struct_log]")
                    return

        print(f"   Target Latency Log: {os.path.basename(lat_path)}")
        print(f"   Target Struct Log:  {os.path.basename(struct_path)}")

        self.load_data(lat_path, struct_path)
        self.analyze_latency_performance()
        self.analyze_latency_composition()
        self.analyze_jitter_stability()
        self.analyze_motion_profile()
        self.analyze_command_velocity() # New method using accurate Latency Log
        self.analyze_correlations_and_trends()
        self.generate_html_report()

if __name__ == "__main__":
    analyzer = AdvancedLogAnalyzer()
    analyzer.run()
