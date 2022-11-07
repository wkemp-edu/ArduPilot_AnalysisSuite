#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 23 19:17:12 2022

@author: williamkemp
"""

class airplane:
    
    def __init__(self, mass, chord, span):
        self.mass = mass
        self.weight = self.mass * 9.807
        
        self.chord = chord
        self.span = span
        self.area = chord * span
        self.AR = (self.span**2) / self.area