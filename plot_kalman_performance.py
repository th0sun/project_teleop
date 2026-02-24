import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os

def plot_kalman_performance(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: Could not find file {csv_file}")
        return

    # Load data
    df = pd.read_csv(csv_file)
    
    # Check if the file has data
    if len(df) < 2:
        print(f"File {csv_file} does not contain enough data to plot.")
        return

    # Normalize timestamps to start at 0
    t = df['Timestamp_ROS'] - df['Timestamp_ROS'].iloc[0]

    # Create a 2x2 grid of plots for the 4 joints
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Kalman Filter Target Prediction\n{csv_file}', fontsize=16)

    joints = ['J1', 'J2', 'J3', 'J4']
    colors_raw = ['red', 'green', 'blue', 'orange']
    colors_pred = ['darkred', 'darkgreen', 'darkblue', 'darkorange']

    for i, ax in enumerate(axes.flatten()):
        joint = joints[i]
        raw_col = f'Raw_{joint}'
        pred_col = f'Pred_{joint}'

        # Plot raw Unity target (Noisy/Delayed)
        ax.plot(t, df[raw_col], label=f'{joint} Raw (Unity)', 
                color=colors_raw[i], alpha=0.5, linestyle=':', linewidth=1)
        
        # Plot Kalman Predicted target
        ax.plot(t, df[pred_col], label=f'{joint} Predicted (Filter)', 
                color=colors_pred[i], alpha=0.9, linewidth=2)

        ax.set_title(f'Joint {i+1} Target Tracking')
        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Angle (rad)')
        ax.legend()
        ax.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save the plot
    png_filename = csv_file.replace('.csv', '_plot.png')
    plt.savefig(png_filename)
    print(f"✅ Plot saved to {png_filename}")
    
    # Show interactive plot
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot Kalman Filter performance from teleop CSV logs')
    parser.add_argument('csv_file', type=str, help='Path to the teleop_analytics_*.csv file')
    args = parser.parse_args()
    
    plot_kalman_performance(args.csv_file)
