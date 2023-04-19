import matplotlib.pyplot as plt
import numpy as np

# Close all open figures
plt.close('all')


def sbmpo_stats(stats_file):
    """
    Convert stats CSV to Python data

    Parameters:
        stats_file (str): path to the CSV file containing stats data

    Returns:
        stats (list of dicts): list of dictionaries representing stats data
    """

    # Read the CSV file
    csv_matrix = np.genfromtxt(stats_file, delimiter=',')

    # Initialize the list of stats dictionaries
    stats = []

    # Convert each row of the CSV to a dictionary and append to the list
    for row in csv_matrix:
        stats.append({
            'time_us': int(row[0]),
            'exit_code': int(row[1]),
            'iterations': int(row[2]),
            'cost': float(row[3]),
            'buffer_size': int(row[4]),
            'success_rate': float(row[5])
        })

    return stats

def sbmpo_results(results_file):
    # Load the CSV file
    csv_matrix = np.loadtxt(results_file, delimiter=',')

    # Initialize output variables
    paths = []
    nodes = []

    # Parse the data
    p = 0
    for line in range(0, csv_matrix.shape[0], 2):
        plan_data = csv_matrix[line,:]

        # Process the path data
        path = {
            'path_size': plan_data[0],
            'num_states': plan_data[1],
            'num_controls': plan_data[2],
            'nodes': []
        }
        i = 3
        for n in range(path['path_size']):
            node_data = {
                'generation': plan_data[i+1],
                'f': plan_data[i+2],
                'g': plan_data[i+3],
                'rhs': plan_data[i+4],
                'state': plan_data[i+5:i+5+path['num_states']]
            }
            i += 5 + path['num_states']
            if n < path['path_size'] - 1:
                node_data['control'] = plan_data[i:i+path['num_controls']]
                i += path['num_controls']
            path['nodes'].append(node_data)
        paths.append(path)

        # Process the nodes data
        plan_data = csv_matrix[line+1,:]
        node_data = {
            'buffer_size': plan_data[0],
            'num_states': plan_data[1],
            'nodes': []
        }
        i = 2
        for n in range(node_data['buffer_size']):
            node = {
                'generation': plan_data[i+1],
                'f': plan_data[i+2],
                'g': plan_data[i+3],
                'rhs': plan_data[i+4],
                'state': plan_data[i+5:i+5+node_data['num_states']]
            }
            i += 5 + node_data['num_states']
            node_data['nodes'].append(node)
        nodes.append(node_data)

    return paths, nodes


# Load data
#stats = sbmpo_stats("/home/camilo/Desktop/sbmpo-momentum/my_project_ws/my_project/csv/stats.csv")
path, nodes = sbmpo_results("/home/camilo/Desktop/sbmpo-momentum/my_project_ws/my_project/csv/nodes.csv")

# Path
t = [0] * path.path_size
x = [0] * path.path_size
v = [0] * path.path_size
u = [0] * (path.path_size-1)

for nd in range(path.path_size):
    t[nd] = path.nodes[nd].g
    x[nd] = path.nodes[nd].state[0]
    v[nd] = path.nodes[nd].state[1]
    
    if (nd < path.path_size-1):
        u[nd] = path.nodes[nd].control[0]
    else:
        u[nd] = float('nan')

# Plot X vs T
plt.figure()
plt.subplot(2, 2, (1, 3))
plt.plot(t, x)
plt.title("Position")
plt.xlabel("Time (s)")
plt.ylabel("X (m)")

# Plot V vs T
plt.subplot(2, 2, 2)
plt.plot(t, v)
plt.title("Velocity")
plt.xlabel("Time (s)")
plt.ylabel("V (m/s)")

# Plot U vs T
plt.subplot(2, 2, 4)
plt.plot(t, u)
plt.title("Control")
plt.xlabel("Time (s)")
plt.ylabel("U (m/s^2)")

# State space
x_all = [0] * nodes.buffer_size
v_all = [0] * nodes.buffer_size

for nd in range(nodes.buffer_size):
    x_all[nd] = nodes.nodes[nd].state[0]
    v_all[nd] = nodes.nodes[nd].state[1]

# Plot V vs X
plt.figure()
plt.plot(x_all, v_all, 'ob', markersize=2)
plt.plot(x, v, '-g', linewidth=5)
plt.xlabel("X")
plt.ylabel("V")
plt.title("State space")
plt.axis([-15, 15, -5, 5])

# Time heuristic
v_opt = linspace(-15, 15, 100)
x_opt = -v_opt * abs(v_opt) / (2 * max(u))
plt.plot(x_opt, v_opt, '--k')

plt.show()


