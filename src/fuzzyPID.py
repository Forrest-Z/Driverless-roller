#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
=======================================
Fuzzy Control Systems: Advanced Example
=======================================

The `tipping problem <./plot_tipping_problem_newapi.html>`_ is a classic,
simple example. If you're new to this, start with the `Fuzzy Control Primer
<../userguide/fuzzy_control_primer.html>`_ and move on to the tipping problem.

This example assumes you're familiar with those topics. Go on. We'll wait.


Typical Fuzzy Control System
----------------------------

Many fuzzy control systems are tasked to keep a certain variable close to a
specific value. For instance, the temperature for an industrial chemical
process might need to be kept relatively constant. In order to do this, the
system usually knows two things:

* The `error`, or deviation from the ideal value
* The way the error is changing. This is the mathematical first derivative;
  we'll call it `delta`

From these two values we can construct a system which will act appropriately.


Set up the Fuzzy Control System
-------------------------------

We'll use the new control system API for this problem. It would be far too
complicated to model manually!
"""
import numpy as np
import skfuzzy.control as ctrl
import time

def fuzzyPID(error_in, delta_in):
    # Sparse universe makes calculations faster, without sacrifice accuracy.
    # Only the critical points are included here; making it higher resolution is
    # unnecessary.
    startTime = time.time()
    errUniverse = np.linspace(-0.4, 0.4, 5) # 输入需严格调整到此范围 配合延时时间
    deltaUniverse = np.linspace(-0.035, 0.035, 5) # new - old
    PgainUniverse = np.linspace(0.5, 2., 5)

    # Create the three fuzzy variables - two inputs, one output
    error = ctrl.Antecedent(errUniverse, 'error')
    delta = ctrl.Antecedent(deltaUniverse, 'delta')
    Pgain = ctrl.Consequent(PgainUniverse, 'Pgain')


    # Here we use the convenience `automf` to populate the fuzzy variables with
    # terms. The optional kwarg `names=` lets us specify the names of our Terms.
    names = ['nb', 'ns', 'ze', 'ps', 'pb']
    PgainNames = ['ps', 'pm', 'pb']
    # PgainNames = ['nb', 'ns', 'ze', 'ps', 'pb']
    # names = ['nb', 'nm', 'ns', 'ze', 'ps', 'pm', 'pb']
    '''
    NB=负方向大的偏差(Negative Big)
    NM=负方向中的偏差(Negative Medium)
    NS=负方向小的偏差(Negative Small)
    ZO=近于零的偏差(Zero)
    PS=正方向小的偏差(Positive Small)
    PM=正方向中的偏差(Positive Medium)
    PB=正方向大的偏差(Positive Big)
    '''
    error.automf(names=names)
    delta.automf(names=names)
    Pgain.automf(names=PgainNames)
    """
    Define complex rules
    --------------------
                                  # (error['ns'] & delta['ns']) | # TODO
                                  # (error['ps'] & delta['ps']) |), # TODO
    This system has a complicated, fully connected set of rules defined below.
    """
    # tips: error is nb or pb, then only two choices:  increase or hold gain
    # increase gain (tips: error: nb pb)
    rule0 = ctrl.Rule(antecedent=((error['nb'] & delta['nb']) |
                                  (error['ns'] & delta['nb']) |
                                  (error['nb'] & delta['ns']) |
                                  (error['pb'] & delta['pb']) |
                                  (error['ps'] & delta['pb']) |
                                  (error['pb'] & delta['ps']) |
                                  (error['ns'] & delta['ns']) |
                                  (error['ps'] & delta['ps'])),
                      consequent=Pgain['pb'], label='rule pb')
    # reduce gain (tips: delta: nb pb)
    rule1 = ctrl.Rule(antecedent=((error['ze'] & delta['nb']) |
                                  (error['ps'] & delta['nb']) |
                                  (error['ns'] & delta['pb']) |
                                  (error['ze'] & delta['pb'])),
                      consequent=Pgain['ps'], label='rule ps')
    # hold (tips: error: ze and not delta pb or nb)
    rule2 = ctrl.Rule(antecedent=((error['nb'] & delta['pb']) |
                                  (error['nb'] & delta['ze']) |
                                  (error['nb'] & delta['ps']) |
                                  (error['ns'] & delta['ps']) |
                                  (error['ns'] & delta['ze']) |
                                  (error['ze'] & delta['ze']) |
                                  (error['ze'] & delta['ns']) |
                                  (error['ze'] & delta['ps']) |
                                  (error['ps'] & delta['ns']) |
                                  (error['ps'] & delta['ze']) |
                                  (error['pb'] & delta['nb']) |
                                  (error['pb'] & delta['ze']) |
                                  (error['pb'] & delta['ns'])),
                      consequent=Pgain['pm'], label='rule pm')

    """
    Despite the lengthy ruleset, the new fuzzy control system framework will
    execute in milliseconds. Next we add these rules to a new ``ControlSystem``
    and define a ``ControlSystemSimulation`` to run it.
    """

    system = ctrl.ControlSystem(rules=[rule0, rule1, rule2])
    # Later we intend to run this system with a 21*21 set of inputs, so we allow
    # that many plus one unique runs before results are flushed.
    # Subsequent runs would return in 1/8 the time!
    # sim = ctrl.ControlSystemSimulation(system, flush_after_run=21 * 21 + 1)

    sim = ctrl.ControlSystemSimulation(system, flush_after_run = 2)
    sim.input['error'] = error_in
    sim.input['delta'] = delta_in
    sim.compute()
    Pcoeff = sim.output['Pgain']
    endTime = time.time()
    costTime = endTime - startTime
#    print 'time cost', costTime
#    print 'Pcoeff',Pcoeff
    return Pcoeff
    
if __name__ == "__main__":
    fuzzyPID(-0.3,-1)
"""
View the control space
----------------------

With helpful use of Matplotlib and repeated simulations, we can observe what
the entire control system surface looks like in three dimensions!
"""
'''
# We can simulate at higher resolution with full accuracy
upsampled = np.linspace(-2, 2, 21)
x, y = np.meshgrid(upsampled, upsampled)
z = np.zeros_like(x)

# Loop through the system 21*21 times to collect the control surface
for i in range(21):
    for j in range(21):
        sim.input['error'] = x[i, j]
        sim.input['delta'] = y[i, j]
        sim.compute()
        z[i, j] = sim.output['output']

# Plot the result in pretty 3D with alpha blending
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

surf = ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap='viridis',
                       linewidth=0.4, antialiased=True)

cset = ax.contourf(x, y, z, zdir='z', offset=-2.5, cmap='viridis', alpha=0.5)
cset = ax.contourf(x, y, z, zdir='x', offset=3, cmap='viridis', alpha=0.5)
cset = ax.contourf(x, y, z, zdir='y', offset=3, cmap='viridis', alpha=0.5)

ax.view_init(30, 200)
'''

"""
.. image:: PLOT2RST.current_figure

Final thoughts
--------------

This example used a number of new, advanced techniques which may be helpful in
practical fuzzy system design:

* A highly sparse (maximally sparse) system
* Controll of Term names generated by `automf`
* A long and logically complicated ruleset, with order-of-operations respected
* Control of the cache flushing on creation of a ControlSystemSimulation,
  which can be tuned as needed depending on memory constraints
* Repeated runs of a ControlSystemSimulation
* Creating and viewing a control surface in 3D.
"""
