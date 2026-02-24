import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import io

def clean_and_load_csv(csv_file):
    """
    Cleans up git diff artifacts (like `+`, `@@`) in case the CSV was copy-pasted from a diff viewer,
    and returns a pandas DataFrame.
    """
    with open(csv_file, 'r') as f:
        lines = f.readlines()
        
    cleaned_lines = []
    for line in lines:
        if line.startswith('@@') or line.startswith('+++') or line.startswith('---'):
            continue
        if line.startswith('+') or line.startswith('-'):
            cleaned_lines.append(line[1:])
        else:
            cleaned_lines.append(line)
            
    return pd.read_csv(io.StringIO("".join(cleaned_lines)))

def analyze_kalman_performance(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: Could not find file {csv_file}")
        return

    print(f"Loading and processing {csv_file}...")
    df = clean_and_load_csv(csv_file)
    
    if len(df) < 20:
        print(f"File {csv_file} does not contain enough data to plot.")
        return

    # 1. Normalize time to start at 0
    df['t_ros'] = df['Timestamp_ROS'] - df['Timestamp_ROS'].iloc[0]
    df['t_unity'] = df['Timestamp_Unity'] - df['Timestamp_ROS'].iloc[0]

    # 2. Create Plot
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(2, 2)
    axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), 
            fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]
    
    fig.suptitle(f'Deep Analysis: Kalman Predictive Filter vs Raw Network Jitter\n{csv_file}', fontsize=18, fontweight='bold', y=0.98)

    joints = ['J1', 'J2', 'J3', 'J4']
    colors_raw = ['#ff7f0e', '#2ca02c', '#1f77b4', '#d62728']
    colors_pred = ['#8c564b', '#006400', '#00008b', '#8b0000']

    # 3. Find Active Region (Zoom in to where movement happens)
    movement_variance = df[['Raw_J1', 'Raw_J2', 'Raw_J3', 'Raw_J4']].var()
    most_active_joint = joints[movement_variance.argmax()]
    
    vel = np.abs(np.gradient(df[f'Raw_{most_active_joint}'], df['t_ros']))
    moving_indices = np.where(vel > 0.05)[0]
    
    if len(moving_indices) > 50:
        start_idx = max(0, moving_indices[0] - 50)
        end_idx = min(len(df)-1, moving_indices[-1] + 150)
        df_zoom = df.iloc[start_idx:end_idx].copy()
        print(f"Zooming into active region: from t={df_zoom['t_ros'].iloc[0]:.2f}s to t={df_zoom['t_ros'].iloc[-1]:.2f}s")
    else:
        df_zoom = df.copy()

    # 4. Plotting loop
    for i, ax in enumerate(axes):
        joint = joints[i]
        raw_col = f'Raw_{joint}'
        pred_col = f'Pred_{joint}'

        # Plot 1: Raw Packets (Scatter to show latency/jitter gaps)
        ax.scatter(df_zoom['t_ros'], df_zoom[raw_col], color=colors_raw[i], s=15, alpha=0.6, 
                   label=f'{joint} Raw Packets (Arrival)', marker='o', zorder=2)
        
        # Plot 2: Raw Line (Dotted to show steps)
        ax.plot(df_zoom['t_ros'], df_zoom[raw_col], color=colors_raw[i], alpha=0.3, 
                linestyle='--', linewidth=1, zorder=1)

        # Plot 3: Kalman Prediction (Thick and Smooth)
        ax.plot(df_zoom['t_ros'], df_zoom[pred_col], color=colors_pred[i], 
                linewidth=3.5, alpha=0.9, label=f'{joint} Kalman Predicted (Smooth)', zorder=3)

        # Difference
        pred_diff_deg = np.degrees((np.abs(df_zoom[pred_col] - df_zoom[raw_col])).mean())

        ax.set_title(f'{joint} Tracking (Avg Jitter Filter: {pred_diff_deg:.2f}°)', fontsize=14, fontweight='bold', pad=10)
        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Joint Angle (Radians)', fontsize=12)
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, linestyle=':', alpha=0.7)

        # Dynamic Y Limit zooming focused on active data
        margin = (df_zoom[pred_col].max() - df_zoom[pred_col].min()) * 0.1
        if margin > 0:
            ax.set_ylim(df_zoom[pred_col].min() - margin, df_zoom[pred_col].max() + margin)

    textstr = '\n'.join((
        "How to read this chart:",
        "1. Orange/Colored Dots = Packet arrival from Unity.",
        "   - Notice the 'staircase' effect due to network latency/jitter.",
        "2. Dark Thick Line = Kalman Filter Prediction.",
        "   - Notice the smooth curve that fills in gaps and predicts ahead."
    ))
    props = dict(boxstyle='round', facecolor='white', alpha=0.8, edgecolor='gray')
    fig.text(0.5, 0.05, textstr, fontsize=12, ha='center', va='bottom', bbox=props, family='monospace')

    plt.tight_layout(rect=[0, 0.10, 1, 0.95], h_pad=3.0, w_pad=2.0)
    
    png_filename = csv_file.replace('.csv', '_analysis.png')
    plt.savefig(png_filename, dpi=150)
    print(f"✅ Advanced Analysis Plot saved to {png_filename}")
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Deep Analysis of Kalman Filter')
    parser.add_argument('csv_file', type=str, help='Path to the teleop_analytics_*.csv file')
    args = parser.parse_args()
    analyze_kalman_performance(args.csv_file)
