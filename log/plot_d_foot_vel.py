import matplotlib.pyplot as plt
import numpy as np
import os

file_path = "d_foot_vel.txt"

def read_data(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    
    left_data = []   # left leg (flag 0)
    right_data = []  # right leg (flag 1)
    left_indices = []  # left leg data indices
    right_indices = [] # right leg data indices
    
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file):
            if not line.strip():
                continue
                
            parts = line.split()
            if len(parts) < 3:
                print(f"Warning: Skipping line {line_number+1} - only {len(parts)} columns")
                continue
                
            try:
                flag = int(parts[1])
                value = float(parts[2])
                
                if flag == 0:
                    left_data.append(value)
                    left_indices.append(line_number)
                elif flag == 1:
                    right_data.append(value)
                    right_indices.append(line_number)
                else:
                    print(f"Warning: Invalid flag value {flag} on line {line_number+1}")
                    
            except ValueError:
                print(f"Skipping invalid line {line_number+1}: {line.strip()}")
    
    if not left_data and not right_data:
        raise ValueError("No valid data found in the file")

    return (
        np.array(left_indices), 
        np.array(left_data), 
        np.array(right_indices), 
        np.array(right_data)
    )

def plot_data(left_indices, left_data, right_indices, right_data):

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax1.plot(left_indices, left_data, 
             'b-', linewidth=1.5, 
             label='Left Leg Data')
    
    ax1.set_title('Left Leg Data (Flag=0)')
    ax1.set_ylabel('Value')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend(loc='best')
    
    ax2.plot(right_indices, right_data, 
             'r-', linewidth=1.5, 
             label='Right Leg Data')
    
    ax2.set_title('Right Leg Data (Flag=1)')
    ax2.set_xlabel('Data Index (Line Number)')
    ax2.set_ylabel('Value')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(loc='best')
    
    plt.tight_layout()
    plt.subplots_adjust(hspace=0.3)
    plt.show()

if __name__ == "__main__":
    try:
        left_indices, left_data, right_indices, right_data = read_data(file_path)
        plot_data(left_indices, left_data, right_indices, right_data)
    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)
