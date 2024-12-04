import attitude_visualization
import serial_read
from math import pi

from data_plotter import plot_in_one_figure
import matplotlib.pyplot as plt
import folium
import webbrowser

# make sure you have different labels for data input and filter output

testDataReader = serial_read.Read("test_data\\imu_gps.csv")
filterDataReader = serial_read.Read("test_data\\output.csv")
indices = {}
rad_or_deg = {"filter": "rad", "gps_update": "deg", "map": "deg", "graph": "deg"} # do not change filter or map units

def convert_angle(input_type, output_type):
    if rad_or_deg[input_type] == "rad" and rad_or_deg[output_type] == "rad":
        return 1
    elif rad_or_deg[input_type] == "rad" and rad_or_deg[output_type] == "deg":
        return 180 / pi
    elif rad_or_deg[input_type] == "deg" and rad_or_deg[output_type] == "rad":
        return pi / 180
    else: 
        return 1

headers = testDataReader.getData()
i = 0
for head in headers:
    name = head.split()[0]
    indices[name] = i
    try:
        unit = head.split()[1]
        if "rad" in unit:
            rad_or_deg[name] = "rad"
        elif "deg" in unit:
            rad_or_deg[name] = "deg"
    except:
        pass
    i += 1 

headers = filterDataReader.getData()
i = 0
for head in headers:
    name = head.split()[0]
    indices[name] = i
    try:
        unit = head.split()[1]
        if "rad" in unit:
            rad_or_deg[name] = "rad"
        elif "deg" in unit:
            rad_or_deg[name] = "deg"
    except:
        pass
    i += 1 

dataLine = list(map(float, testDataReader.getData())) # list of data (string parsed and split in reader)
filterLine = list(map(float, filterDataReader.getData()))

drawer = attitude_visualization.Display()

algo_data = {"p": [], "v": [], "q": [], "raw_gyro": [], "raw_accel": [], "raw_pos": [], "raw_vel": []}
time = []
algo_points = []
ref_points = []
gps_points = []

while True: 
    print(dataLine[indices["time"]])
    time.append(dataLine[indices["time"]])
    q = [filterLine[indices["filter_q0"]], filterLine[indices["filter_q1"]], filterLine[indices["filter_q2"]], filterLine[indices["filter_q3"]]]
    p = [filterLine[indices["filter_lat"]], filterLine[indices["filter_lon"]], filterLine[indices["filter_alt"]]]
    drawer.draw(q[0], q[1], q[2], q[3], p[2])
    algo_points.append((convert_angle("filter", "map") * p[0], convert_angle("filter", "map") * p[1]))
    ref_points.append((convert_angle("ref_pos_lat", "map") * dataLine[indices["ref_pos_lat"]], convert_angle("ref_pos_lon", "map") * dataLine[indices["ref_pos_lon"]]))
    gps_points.append((convert_angle("gps_lat", "map") * dataLine[indices["gps_lat"]], convert_angle("gps_lon", "map") * dataLine[indices["gps_lon"]])) # fixed error here
    v = [filterLine[indices["filter_vN"]], filterLine[indices["filter_vE"]], filterLine[indices["filter_vD"]]]
    algo_data["p"].append([convert_angle("filter", "graph") * p[0], convert_angle("filter", "graph") * p[1], p[2]])
    algo_data["v"].append(v)
    algo_data["q"].append(q)
    algo_data["raw_gyro"].append([convert_angle("gyro_x", "graph") * dataLine[indices["gyro_x"]], convert_angle("gyro_y", "graph") * dataLine[indices["gyro_y"]], \
                                  convert_angle("gyro_z", "graph") * dataLine[indices["gyro_z"]]])
    algo_data["raw_accel"].append([dataLine[indices["accel_x"]], dataLine[indices["accel_y"]], dataLine[indices["accel_z"]]])
    algo_data["raw_pos"].append([convert_angle("gps_lat", "graph") * dataLine[indices["gps_lat"]], \
                                 convert_angle("gps_lon", "graph") * dataLine[indices["gps_lon"]], dataLine[indices["gps_alt"]]])
    algo_data["raw_vel"].append([dataLine[indices["gps_vN"]], dataLine[indices["gps_vE"]], dataLine[indices["gps_vD"]]])

    try:
        dataLine = list(map(float, testDataReader.getData())) 
        filterLine = list(map(float, filterDataReader.getData()))
    except:
        break

position = []
velocity = []
for i in range(len(algo_data["p"])):
    position.append(algo_data["raw_pos"][i]+algo_data["p"][i])
    velocity.append(algo_data["raw_vel"][i]+algo_data["v"][i])
plot_in_one_figure(time, position, title="Position", legend=("raw lat (deg)", "raw lon (deg)", "raw alt (m)", \
                                                             "filtered lat (deg)", "filtered lon (deg)", "filtered alt (m)"))
plot_in_one_figure(time, velocity, title="Velocity (NED)", legend=("raw vN (m/s)", "raw vE (m/s)", "raw vD (m/s)", \
                                                                   "filtered vN (m/s)", "filtered vE (m/s)", "filtered vD (m/s)"))
plot_in_one_figure(time, algo_data["q"], title="Attitude")
plot_in_one_figure(time, algo_data["raw_accel"], title="Raw Accelerometer", legend=("ax (m/s^2)", "ay (m/s^2)", "az (m/s^2)"))
plot_in_one_figure(time, algo_data["raw_gyro"], title="Raw Gyrometer", legend=("gx (deg/s)", "gy (deg/s)", "gz (deg/s)"))


m = folium.Map(location=algo_points[0], zoom_start=18)
tile = folium.TileLayer(
        tiles = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr = 'Esri',
        name = 'Esri Satellite',
        overlay = False,
        control = True
       ).add_to(m)
folium.PolyLine(gps_points, tooltip="Coast", color="green").add_to(m)
folium.PolyLine(ref_points, tooltip="Coast", color="red").add_to(m)
folium.PolyLine(algo_points, tooltip="Coast").add_to(m)
m.save("python_sim\\map\\index.html")

webbrowser.open("python_sim\\map\\index.html")

plt.show()