set term x11

# Specify the file path
file_path = ARG1
data = sprintf('< tail -n 1000 %s', file_path)

set datafile separator ","

set title "Driver response"
set xlabel "Time"
set ylabel "RPM"

set xdata time
set timefmt "%S"
set xtics format "%M:%S"

# Plot the data
plot data u 1:10 w l t "RPM Feedback", data u 1:12 w l t "RPM Setpoint"
reread