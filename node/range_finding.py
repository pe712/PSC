
#coding: utf-8
from math import atan2
import matplotlib.pyplot as plt
import random as rd
from time import time

class range_finding:
    """Traite la donnée ranges de /scan pour fournir le gap optimal
    """
    def __init__(self, ranges, desired_width):
        self.desired_width=desired_width
        self.ranges = ranges
        self.sorted_ranges = sorted(enumerate(ranges), key=lambda x: x[1], reverse=True)   # type: ignore
        # sorted de manière décroissante par distance -> on cherche la plus grande distance
    
    """Renvoie le gap optimal de range
    Le gap optimal est le plus lointain (en prenant la distance minimale du gap) ET d'au moins la largeur desired_width
    Renvoie l'index de début et de fin du gap et sa profondeur
    """
    def find_range(self):
        for index_range, distance in self.sorted_ranges:
            # min_range_count = int(atan2(desired_width, distance)/data.angle_increment)
            min_range_count = int(80/distance)
            gap_range = self.possible_range(index_range, min_range_count, distance) 
            if gap_range is not None:
                return gap_range+ (distance,)
        raise Exception("Impossible to find optimal gap")

    """Renvoie le plus grand gap possible autour de index_range
    Il doit être plus profond que distance, et plus large en nombre d'indices que min_range_count
    Si impossible, renvoie None
    """
    def possible_range(self, index_range, min_range_count, distance):
        start = self.ending_point(-1, index_range, min_range_count, distance)
        stop = self.ending_point(1, index_range, min_range_count, distance)
        if stop-start>min_range_count:
            return (start, stop)
        else:
            return None

    def ending_point(self, sign, index_range, min_range_count, distance):
        if sign==1:
            t_stop = min(len(ranges), index_range+min_range_count*sign)
        else:
            t_stop = max(0, index_range+min_range_count*sign)
        for t in range(index_range, t_stop, sign):
            if ranges[t]<distance:
                return t-sign
        return t_stop


    
if __name__=="__main__":
    
    # desired_width = 3 * car_width
        
    start =time()
    for k in range(1000):
        rf = range_finding([rd.random()*20 for k in range(200)], 0)
        try:
            best_range = rf.find_range()
        except Exception:
            pass
    end = time()
    print("The time of execution of above program is :",(end-start), "ms")


    n=200
    ranges=[rd.random()*20 for k in range(n)]
    rf = range_finding(ranges, 0)
    try:
        a, b, distance = rf.find_range()
        fig, ax = plt.subplots()
        ax.plot(range(n), ranges, color = "green")
        ax.plot(range(a, b), ranges[a:b], color="red")
        plt.show()
        print(ranges[a:b], distance)
    except Exception:
        pass
