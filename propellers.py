#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 03:11:21 2022

@author: williamkemp
"""

import numpy as np
from matplotlib import pyplot as plt

# Updated on October 20th 2022
# Based on curve fits in Createv/Analysis/ file system
# Based on data from: (Yoke drag from 2022-04-18, 2022-04-13)
    # 20 x 8
    # * 2022-05-12
    # * 2022-04-19
    # * 2022-04-14
    # * 2022-04-13
    
    # 18.5 x 12
    # * 2022-05-07
    # * 2022-04-19
    # * 2021-09-10

class aeronaut20x8:
    
    def __init__(self):
        self.diameter = 0.512 # meters

    def efficiency(self, J):
        # Overall efficiency for a 20x8 system, from CP and CT fits
        eff = J * (self.thrust_coeff(J)/self.power_coeff(J))
        return eff
    
    def thrust_coeff(self, J):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        #ct = 0.2948*J**3 - 0.4303*J**2 + 0.02295*J**1 + 0.07395  Old Fit
        CT = (0.1298*J**3) - 0.2679*J**2 - 0.02553*J + 0.07525
        return CT
    
    def power_coeff(self, J):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        CP = -0.1618*J**3 + 0.0292*J**2 - 0.002126*J + 0.02489
        return CP

    def freewheel_tcoeff(self):
        # Finding the drag of a freewheeling propeller by CP = 0, getting J and then finding CT(J)
        
        # Newton Ralphsen Method
        J_old = 0.5        # Initial guess of J
        dJ = 0.01
        error_dem = 1e-5
        cp = self.power_coeff(J_old)
        while cp > error_dem:
            slope = (self.power_coeff(J_old + (0.5*dJ)) - self.power_coeff(J_old - (0.5*dJ))) / dJ
            J_new = J_old - ( self.power_coeff(J_old) / slope)
            cp = self.power_coeff(J_new)
            J_old = J_new
        print(self.power_coeff(J_new))

        return self.thrust_coeff(J_old)

    def getRPM(self, thrust, rho, V_tas):
        # Finding the RPM required for certain thrust @ true airspeed

        # Newton Ralphsen Method
        n_old = 40          # Initial guess of rev/s
        J_old = V_tas / (n_old * self.diameter)
        dn = 0.2            # Incremental change to rev/s

        error_dem = 1e-5
        T_act = self.thrust_coeff(V_tas / (n_old * self.diameter)) * rho * n_old**2 * self.diameter**4
        error = np.abs(thrust - T_act)
        
        while error > error_dem:
            f_old = (self.thrust_coeff(V_tas / ((n_old) * self.diameter)) * rho * (n_old)**2 * self.diameter**4)
            slope = ((self.thrust_coeff(V_tas / ((n_old-0.5*dn) * self.diameter)) * rho * (n_old-0.5*dn)**2 * self.diameter**4) \
                - (self.thrust_coeff(V_tas / ((n_old+0.5*dn) * self.diameter)) * rho * (n_old+0.5*dn)**2 * self.diameter**4)) * dn**-1
            n_new = n_old - ((thrust-f_old)/slope)
            T_act = self.thrust_coeff(V_tas / (n_new * self.diameter)) * rho * n_new**2 * self.diameter**4
            error = np.abs(thrust - T_act)
            n_old = n_new
            print(n_new)
        return n_new
    
    def getTorque(self, rho, n, V_tas):
        # Finding torque from propeller RPM and Airspeed

        J = V_tas / (n * self.diameter)
        CQ = self.power_coeff(J) * (2 * np.pi)**-1
        Q = CQ * rho * n**2 * self.diameter**5
        return Q
    
    def diameter(self):
        return self.diameter

class aeronaut185x12:
    
    def __init__(self):
        self.diameter = 0.4699 # meters

    def efficiency(self, J):
    # Overall efficiency for a 18.5x12 system, with a polynomial fit, find details in onenote
        eff = J * (self.thrust_coeff(J)/self.power_coeff(J))
        return eff
    
    def thrust_coeff(self, J, oldfit=False):
    # Overall thrust coefficient for a 18.5x12 system with a polynomial fit  
        #ct = -0.02627*J**3 - 0.1681*J**2 + 0.02741*J**1 + 0.08628 (original)
        if oldfit:
            CT = 0.01649*J**3 - 0.224*J**2 + 0.04329*J + 0.08563 #(10-18 Fit)
        else:
            CT = -1.636*J**5 + 3.933*J**4 - 3.246*J**3 + 0.8995*J**2 - 0.09467*J**1 + 0.08651 # (11-18 Fit, Freewheel data)
        return CT
    
    def power_coeff(self, J,  oldfit=False):
        # Overall thrust coefficient for a 20x8 system with a polynomial fit  
        if oldfit:
            CP = -0.2103*J**3 + 0.1318*J**2 - 0.033076*J + 0.04726 #(10-18 Fit)
        else:
            CP = 0.2741*J**4 - 0.5853*J**3 + 0.3012*J**2 - 0.05987*J**1 + 0.04802 # (11-18 Fit, Freewheel data)
        return CP

    def freewheel_tcoeff(self):
        # Finding the drag of a freewheeling propeller by CP = 0, getting J and then finding CT(J)
        
        # Newton Ralphsen Method
        J_old = 0.5        # Initial guess of J
        dJ = 0.001
        error_dem = 1e-9
        cp = self.power_coeff(J_old)

        plt.figure()
        Jv = np.linspace(0.1, 1.4, 200)
        plt.plot(Jv, self.power_coeff(Jv))

        while np.abs(cp) > error_dem:
            slope = (self.power_coeff(J_old + (0.5*dJ)) - self.power_coeff(J_old - (0.5*dJ))) / dJ
            J_new = J_old - 0.5*( self.power_coeff(J_old) / slope)
            cp = self.power_coeff(J_new)
            J_old = J_new
            plt.scatter(J_new, self.power_coeff(J_new), marker='o')
        print("Freewheel thrust coefficient: ", str(self.thrust_coeff(J_new)), "\n Advance Ratio: ", str(J_new), "\n CP Freewheel: ", str(cp))
        
        plt.scatter(J_new, self.power_coeff(J_new), marker='^')
        plt.xlabel("Advance Ratio")
        plt.ylabel("Coefficient of Power")
        plt.title("Convergence for Freewheeling Drag Estimation")
        plt.show()

        return self.thrust_coeff(J_old)
    
    def getRPM(self, thrust, rho, V_tas):
        # Finding the RPM required for certain thrust @ true airspeed

        # Newton Ralphsen Method
        n_old = 40          # Initial guess of rev/s
        J_old = V_tas / (n_old * self.diameter)
        dn = 0.2            # Incremental change to rev/s

        error_dem = 1e-5
        T_act = self.thrust_coeff(V_tas / (n_old * self.diameter)) * rho * n_old**2 * self.diameter**4
        error = np.abs(thrust - T_act)
        
        while error > error_dem:
            f_old = (self.thrust_coeff(V_tas / ((n_old) * self.diameter)) * rho * (n_old)**2 * self.diameter**4)
            slope = ((self.thrust_coeff(V_tas / ((n_old+0.5*dn) * self.diameter)) * rho * (n_old+0.5*dn)**2 * self.diameter**4) \
                - (self.thrust_coeff(V_tas / ((n_old-0.5*dn) * self.diameter)) * rho * (n_old-0.5*dn)**2 * self.diameter**4)) * dn**-1
            n_new = n_old - (f_old/slope)
            T_act = self.thrust_coeff(V_tas / (n_new * self.diameter)) * rho * n_new**2 * self.diameter**4
            error = np.abs(thrust - T_act)
            n_old = n_new
            print(n_new)
        return n_new

    def getTorque(self, rho, n, V_tas):
        # Finding torque from propeller RPM and Airspeed

        J = V_tas / (n * self.diameter)
        CQ = self.power_coeff(J) * (2 * np.pi)**-1
        Q = CQ * rho * n**2 * self.diameter**5
        return Q

    def diameter(self):
        return self.diameter
    
class aeronaut11x7_estimatedBEN:
    
    def __init__(self):
        self.diameter = 0.288 # meters (This is slightly inconsistent with the diameter used by Ben 279.4 mm)

    def efficiency(self, J):
    # Overall efficiency for a 18.5x12 system, with a polynomial fit, find details in onenote
        eff = J * (self.thrust_coeff(J)/self.power_coeff(J))
        return eff
    
    def thrust_coeff(self, J, oldfit=False):
        CT = 0.10244 - 0.0023939*J - 0.22146*J**2 + 0.047003*J**3 + 0.038813*J**4
        return CT
    
    def power_coeff(self, J,  oldfit=False):
        CP = 0.040615 - 0.01124*J + 0.17948*J**2 - 0.47695*J**3 + 0.26034*J**4
        return CP

    def freewheel_tcoeff(self):
        # Finding the drag of a freewheeling propeller by CP = 0, getting J and then finding CT(J)
        
        # Newton Ralphsen Method
        J_old = 0.5        # Initial guess of J
        dJ = 0.001
        error_dem = 1e-9
        cp = self.power_coeff(J_old)

        plt.figure()
        Jv = np.linspace(0.1, 1.4, 200)
        plt.plot(Jv, self.power_coeff(Jv))

        while np.abs(cp) > error_dem:
            slope = (self.power_coeff(J_old + (0.5*dJ)) - self.power_coeff(J_old - (0.5*dJ))) / dJ
            J_new = J_old - 0.5*( self.power_coeff(J_old) / slope)
            cp = self.power_coeff(J_new)
            J_old = J_new
            plt.scatter(J_new, self.power_coeff(J_new), marker='o')
        print("Freewheel thrust coefficient: ", str(self.thrust_coeff(J_new)), "\n Advance Ratio: ", str(J_new), "\n CP Freewheel: ", str(cp))
        
        plt.scatter(J_new, self.power_coeff(J_new), marker='^')
        plt.xlabel("Advance Ratio")
        plt.ylabel("Coefficient of Power")
        plt.title("Convergence for Freewheeling Drag Estimation")
        plt.show()

        return self.thrust_coeff(J_old)
    
    def getRPM(self, thrust, rho, V_tas):
        # Finding the RPM required for certain thrust @ true airspeed

        # Newton Ralphsen Method
        n_old = 40          # Initial guess of rev/s
        J_old = V_tas / (n_old * self.diameter)
        dn = 0.2            # Incremental change to rev/s

        error_dem = 1e-5
        T_act = self.thrust_coeff(V_tas / (n_old * self.diameter)) * rho * n_old**2 * self.diameter**4
        error = np.abs(thrust - T_act)
        
        while error > error_dem:
            f_old = (self.thrust_coeff(V_tas / ((n_old) * self.diameter)) * rho * (n_old)**2 * self.diameter**4)
            slope = ((self.thrust_coeff(V_tas / ((n_old+0.5*dn) * self.diameter)) * rho * (n_old+0.5*dn)**2 * self.diameter**4) \
                - (self.thrust_coeff(V_tas / ((n_old-0.5*dn) * self.diameter)) * rho * (n_old-0.5*dn)**2 * self.diameter**4)) * dn**-1
            n_new = n_old - (f_old/slope)
            T_act = self.thrust_coeff(V_tas / (n_new * self.diameter)) * rho * n_new**2 * self.diameter**4
            error = np.abs(thrust - T_act)
            n_old = n_new
            print(n_new)
        return n_new

    def getTorque(self, rho, n, V_tas):
        # Finding torque from propeller RPM and Airspeed

        J = V_tas / (n * self.diameter)
        CQ = self.power_coeff(J) * (2 * np.pi)**-1
        Q = CQ * rho * n**2 * self.diameter**5
        return Q

    def diameter(self):
        return self.diameter