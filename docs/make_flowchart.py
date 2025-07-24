#!/usr/bin/python3

from graphviz import Digraph

# Create directed graph
fc = Digraph(
    name='SolarBMS',
    format='svg',
    graph_attr={'rankdir': 'TB', 'splines': 'spline', 'nodesep': '0.2'},
    node_attr={'shape': 'box', 'style': 'rounded,filled', 'fillcolor': '#e0e0e0'},
    edge_attr={'arrowsize': '0.7'}
)

# Define nodes with custom styles
fc.node('START', 'Start', shape='ellipse', fillcolor='#aaffaa')
fc.node('CHECK_UTIL', 'Utility\npresent?', shape='diamond', fillcolor='#ffcccb')
fc.node('CHECK_SUN', 'Sunlight\npresent?', shape='diamond', fillcolor='#ffcccb')
fc.node('BAT_ABOVE_12V_1', 'Battery > 12V?', shape='diamond', fillcolor='#ffcccb')
fc.node('WAIT_SINCE_BAT_OVERLOAD_UND_VOLT', 'Wait 5 min since\nbattery overloaded\nor under-voltage',
        fillcolor='#e1bee7')
fc.node('WAIT_SINCE_INV_TURNED_OFF', 'Wait 5 min since\ninverter turned off', fillcolor='#e1bee7')
fc.node('SW_TO_UTIL', 'Switch to utility\nand record reason', fillcolor='#bbdefb')
fc.node('SW_TO_INV', 'Switch to\ninverter', fillcolor='#bbdefb')

fc.node('BAT_ABOVE_12V_2', 'Battery > 12V?', shape='diamond', fillcolor='#ffcccb')
fc.node('BAT_ABOVE_11V', 'Battery > 11V?', shape='diamond', fillcolor='#ffcccb')
fc.node('WAIT_BW_11V_12V', 'Wait\n2 min', fillcolor='#e1bee7')

fc.node('DRAW_ABOVE_20A', 'Drawing > 20A?', shape='diamond', fillcolor='#ffcccb')
fc.node('DRAW_ABOVE_50A', 'Drawing > 50A?', shape='diamond', fillcolor='#ffcccb')
fc.node('CHECK_UTIL_SUN', 'Sunlight & utility\npresent?', shape='diamond', fillcolor='#ffcccb')
fc.node('DRAW_ABOVE_5A', 'Drawing > 5A?', shape='diamond', fillcolor='#ffcccb')
fc.node('WAIT_10_SEC', 'Wait\n10 sec', fillcolor='#e1bee7')
fc.node('WAIT_5_MIN', 'Wait\n5 min', fillcolor='#e1bee7')
fc.node('WAIT_2_MIN', 'Wait\n2 min', fillcolor='#e1bee7')

fc.edge('START', 'CHECK_UTIL')
fc.edge('CHECK_UTIL', 'BAT_ABOVE_12V_1', label='No')
fc.edge('CHECK_UTIL', 'CHECK_SUN', label='Yes')
fc.edge('BAT_ABOVE_12V_1', 'WAIT_SINCE_BAT_OVERLOAD_UND_VOLT', label='Yes')
fc.edge('CHECK_SUN', 'WAIT_SINCE_INV_TURNED_OFF', label='Yes')
fc.edge('CHECK_SUN', 'SW_TO_UTIL', label='No')
fc.edge('WAIT_SINCE_BAT_OVERLOAD_UND_VOLT', 'SW_TO_INV')
fc.edge('WAIT_SINCE_INV_TURNED_OFF', 'SW_TO_INV')

fc.edge('SW_TO_INV', 'BAT_ABOVE_12V_2')
fc.edge('BAT_ABOVE_12V_2', 'BAT_ABOVE_11V', label='No', color='#ff0000')
fc.edge('BAT_ABOVE_11V', 'WAIT_BW_11V_12V', label='Yes', color='#ff0000')
fc.edge('BAT_ABOVE_11V', 'SW_TO_UTIL', label='No', color='#ff0000')
fc.edge('WAIT_BW_11V_12V', 'SW_TO_UTIL', color='#ff0000')

fc.edge('SW_TO_INV', 'DRAW_ABOVE_20A')
fc.edge('DRAW_ABOVE_20A', 'DRAW_ABOVE_50A', label='Yes', color='#ff0000')
fc.edge('DRAW_ABOVE_20A', 'CHECK_UTIL_SUN', label='No')
fc.edge('DRAW_ABOVE_50A', 'SW_TO_UTIL', label='Yes', color='#ff0000')
fc.edge('DRAW_ABOVE_50A', 'WAIT_10_SEC', label='No', color='#ff0000')
fc.edge('CHECK_UTIL_SUN', 'DRAW_ABOVE_5A', label='Yes')
fc.edge('DRAW_ABOVE_5A', 'WAIT_5_MIN', label='No')
fc.edge('DRAW_ABOVE_5A', 'WAIT_2_MIN', label='Yes')
fc.edge('WAIT_10_SEC', 'SW_TO_UTIL', color='#ff0000')
fc.edge('WAIT_5_MIN', 'SW_TO_UTIL')
fc.edge('WAIT_2_MIN', 'SW_TO_UTIL')

fc.render('flowchart', view=True)
