# Purpose:
# 1. Display all relevant information to CREATeV flight analysis
# 2. Platform to continuously improve data visualization from flight

import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

# General plot:
def general_plot(data):

    # Plotting single figure:
    # 1. Airspeeds (All)
    # 2. Direction (Headings)
    # 3. Roll/Pitch - two y axis plot
    # 4. Inputs - Throttle, elevator, rudder, ailerons

    plt.figure(figsize=(10,5))

    ax0 = plt.subplot(4,1,1)
    plt.plot(data["Airspeed_CTUN"], label="CTUN airspeed (EAS)", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    plt.plot(data["Airspeed_TECS"], label="TECS airspeed (Crazy filtered)", linestyle='--', marker='<', markersize=4, markeredgecolor=[0,0,0])
    plt.plot(data["Airspeed_Sensor0"], label="SPD33 Main Airspeed Sensor", linestyle='--', marker='>', markersize=4, markeredgecolor=[0,0,0])
    plt.plot(data["Airspeed_Sensor1"], label="MS4525 Secondary Airspeed Sensor", linestyle='--', marker='*', markersize=4, markeredgecolor=[0,0,0])
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Airspeeds (m/s)")
    plt.grid(True)

    ax1 = plt.subplot(4,1,2, sharex=ax0)
    plt.legend()
    plt.plot(data["Heading"], label="Heading from Attitude estimation", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    plt.xlabel("Time (s)")
    plt.ylabel("PSI (Degrees)")
    plt.grid(True)

    ax2 = plt.subplot(4,1,3, sharex=ax0)
    ax2_1 = ax2.twinx()
    ax2.plot(data["RollAngle"], label="Roll Angle", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    ax2_1.plot(data["PitchAngle"], label="Pitch Angle", linestyle='--', marker='<', markersize=4, markeredgecolor=[0,0,0])
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Roll & Pitch")
    plt.grid(True)

    ax3 = plt.subplot(4,1,4, sharex=ax0)
    ax3.plot(data["Throttle"], label="Throttle Input", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    ax3.plot(data["Elevator"], label="Elevator Input", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    ax3.plot(data["Rudder"], label="Rudder Input", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    ax3.plot(data["Aileron"], label="Aileron Input", linestyle='--', marker='o', markersize=4, markeredgecolor=[0,0,0])
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Inputs")
    plt.grid(True)

    plt.show()

# Wind plot:
def wind_plot(data):

    # Plotting single figure:
    # 1. Direction of wind
    # 2. Magnitude of wind

    plt.figure(figsize=(10,5))

    ax0 = plt.subplot(2,1,1)
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Wind Direction (Angle from North)")
    plt.grid(True)

    ax1 = plt.subplot(2,1,2, sharex=ax0)
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Wind Magnitude (m/s)")
    plt.grid(True)

    plt.show()

# Roll/Pitch Control Plot:
def rollpitchctrl_plot(data):

    # Plotting single figure:
    # 1. Roll rate & Pitch Rate --> Target actual
    # 2. Demanded roll and pitch
    # 3. Achieved roll and pitch
    # 4. Integrator term

    plt.show()

# TECS plot:
def tecs_plot(data):
    
    # Plotting single figure:
    # 1. Altitudes -> actual & target
    # 2. Airspeeds --> All including demanded TAS EAS
    # 3. Inputs -> Throttle & pitch
    # 4. Flags

    plt.show()

def subplotter(grid=True):



    plt.show()