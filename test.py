import re
import pandas as pd
import matplotlib.pyplot as plt

# Clean up the extracted text and filter only lines with "607a:" followed by a number
lines = extracted_text.splitlines()
filtered_data = []

for line in lines:
    # Check for the pattern "607a: <number>"
    match = re.match(r'607a:\s*(\d+)', line)
    if match:
        # Append the number as an integer
        filtered_data.append(int(match.group(1)))

# Generate a DataFrame with time intervals (4ms per row) and the extracted values
time_intervals = [i * 4 for i in range(len(filtered_data))]  # 4ms interval per data point
data_df = pd.DataFrame({'Time (ms)': time_intervals, 'Value': filtered_data})

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(data_df['Time (ms)'], data_df['Value'], marker='o', linestyle='-', markersize=3)
plt.title('607a Values Over Time')
plt.xlabel('Time (ms)')
plt.ylabel('607a Value')
plt.grid(True)
plt.show()
