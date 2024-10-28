<?xml version="1.0"?>
<multiplot>
    <plot>
        <title>Torque - Joint 1</title>
        <topic>/robot_torque_command</topic>
        <field>torques[0]</field>
        <type>line</type>
        <color>#ff0000</color> <!-- Red color for Joint 1 -->
        <min>0.0</min> <!-- Set to 0 to start y-axis from zero -->
        <max>10.0</max> <!-- Example max value; adjust as needed -->
        <time_window>60.0</time_window> <!-- Keep data for 60 seconds -->
    </plot>
    <plot>
        <title>Torque - Joint 2</title>
        <topic>/robot_torque_command</topic>
        <field>torques[1]</field>
        <type>line</type>
        <color>#00ff00</color> <!-- Green color for Joint 2 -->
        <min>0.0</min> <!-- Set to 0 to start y-axis from zero -->
        <max>10.0</max> <!-- Example max value; adjust as needed -->
        <time_window>60.0</time_window> <!-- Keep data for 60 seconds -->
    </plot>
    <plot>
        <title>Torque - Joint 3</title>
        <topic>/robot_torque_command</topic>
        <field>torques[2]</field>
        <type>line</type>
        <color>#0000ff</color> <!-- Blue color for Joint 3 -->
        <min>0.0</min> <!-- Set to 0 to start y-axis from zero -->
        <max>10.0</max> <!-- Example max value; adjust as needed -->
        <time_window>60.0</time_window> <!-- Keep data for 60 seconds -->
    </plot>
    <x_axis>
        <min>0.0</min> <!-- Force x-axis to start from zero -->
        <time_window>60.0</time_window> <!-- Show data from zero -->
    </x_axis>
</multiplot>
