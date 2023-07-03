# Class definition for particular aircraft
# Internal variables:
# 1. Mass (kg)
# 2. Weight (N), using standard gravitational constant
# 3. Chord (m), geometric mean chord 
# 4. Full wing span (m)
# 5. Wing planform area, calculated from mean chord and wing span
# 6. Aspect ratio, calculated from wing span and wing area

class airplane:
    
    def __init__(self, mass, chord, span):
        self.mass = mass                    # Mass in Kilograms
        self.weight = self.mass * 9.807     # Weight in Newtons
        
        self.chord = chord                  # Geometric mean chord, in meters
        self.span = span                    # Full wing span, in meters
        self.area = chord * span            # Wing planform area, in meters square
        self.AR = (self.span**2) / self.area# Aspect ratio of wing