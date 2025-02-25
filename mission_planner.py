'''
Direct solution for optimal vessel mission planning (<=12 waypoints)

Author: Rachel Mecca

(c) 2024 Regents of the University of Michigan

'''
from datetime import datetime
import numpy as np
from vessel_sim_engine import Waypoint, Destination, Mission, vesselFuel
from geographiclib.geodesic import Geodesic
from itertools import combinations
from tsp_solver import tsp_solver
geod = Geodesic.WGS84
maxsize = float('inf')

class missionPlanner:
    '''
    Finds the optimal solution to the vessel misssion planning problem
    '''
    def __init__(self, vessel,start_time, ocean_file, fuel_constraint=0.1):
        '''
        mission Planner constructor
        ---------------------------------------------------------
        Parameters
        ---------------------------------------------------------
        vessel: vesselFuel object
        start_time: datetime object, starting time of the mission
        ocean_file: string, filepath to ocean dataset
        fuel_constraint: value fuel must be above a valid mission
        '''
        self.vessel = vessel
        self.mission = vessel.mission
        self.start_time=start_time
        #delete end waypoint so it can be added to end later
        del self.mission.destinations[len(self.mission.destinations)-1]
        self.fuel_constraint = fuel_constraint
        self.adj_mat=None
    class waypoint_info:
        '''
        Holds info about distance, time, and fuel_burned between two waypoints
        '''
        def __init__(self, distance, time, fuel_burned):
            '''
            waypoint_info constructor
            ---------------------------------------------------------
            Parameters
            ---------------------------------------------------------
            distance: float, distance between two waypoints
            time: float, number of hours it takes to travel betweeen two waypoints
            fuel_burned: float, amount of fuel burned between two waypoints (kN)
            '''
            self.distance = distance
            self.time = time
            self.fuel_burned = fuel_burned
    def plan_mission(self):
      '''
      finds the mission with the highest point value that meets the constraints
      -------------------------------------------------------------------------
      Returns
      -------------------------------------------------------------------------
      Mission Object: the optimal mission found
      '''
      start=Waypoint(self.vessel.start[0],self.vessel.start[1])
      end=Waypoint(self.vessel.end[0],self.vessel.end[1])
      self.mission.destinations.insert(0,Destination(start,self.mission.destinations[0].arrival_time,0))
      self.adj_mat=self.waypoint_list_to_matrix(self.vessel.mission.destinations,end)
      #check if a route through the original set of waypoints is possible
      solver=tsp_solver(self.vessel,self.mission.destinations,self.adj_mat)
      solver.calc_TSP_opt()
      if(solver.best_path_length<self.vessel.fuel_weight):
          dests=[]
          for i in solver.best_path:
           dests.append(self.mission.destinations[i])  
          dests.append(Destination(end,self.mission.destinations[solver.best_path[len(solver.best_path)-1]].arrival_time,0))        
          return Mission(dests)
      else:
        x=len(self.mission.destinations)-2
        possible_missions=[]
        indexes=list(range(1,len(self.mission.destinations)))
        while(x>0):
           
           destination_combinations=combinations(indexes,x)
           for d in destination_combinations:
               d=(0,) + d
               priority=0
               l=[]
               for i in d:
                   priority+=self.vessel.mission.destinations[i].priority
                   l.append(i)
               possible_missions.append((l,priority))
           x-=1
           #sorts possible missions from highest total priority to lowest total priority
        possible_missions.sort(key=lambda mission: mission[1], reverse=True)
        for mission in possible_missions:
              dest_list=[]
              for i in mission[0]:
                  dest_list.append(self.vessel.mission.destinations[i])
              new_adj=mission[0]+[len(self.vessel.mission.destinations)]
              sub_adj_mat=self.adj_mat[np.ix_(new_adj, new_adj)]
              solver=tsp_solver(self.vessel,dest_list,sub_adj_mat)
              solver.calc_TSP_opt()
              if(solver.best_path_length<self.vessel.fuel_weight):
                  dests=[]
                  for i in solver.best_path:
                   dests.append(dest_list[i])         
                  dests.append(Destination(end,self.mission.destinations[solver.best_path[len(solver.best_path)-1]].arrival_time,0))  
                  return Mission(dests)
        return Mission([Destination(start,datetime.now,0)])   
    def waypoint_list_to_matrix(self, destinations,end):
        '''
        converts a list of destinations to an adjacency matrix
        ------------------------------------------------------
        Parameters
        ------------------------------------------------------
        destinations: list of Destination objects
        ------------------------------------------------------
        Returns
        ------------------------------------------------------
        2d array: adj matrix for destinations
        '''
        matrix = [[None] * (len(destinations)+1) for _ in range(len(destinations)+1)]
        
        for i in range(len(destinations)):
            for j in range(len(destinations)-1,-1,-1):
                if(j<i):
                    break
                if i == j:
                    wpi = self.waypoint_info(0, 0, 0)
                    matrix[i][j]=wpi
                    continue
                g = geod.Inverse(destinations[i].waypoint.lat, destinations[i].waypoint.long,
                                destinations[j].waypoint.lat, destinations[j].waypoint.long)
                distance = g['s12'] / 1852
                speed=self.vessel.speed
                time = distance / speed
                fuel=self.vessel.calc_fuel_burned(self.vessel.speed,time)
                wpi = self.waypoint_info(distance, time, fuel)
                matrix[i][j]=wpi
                matrix[j][i]=wpi
        for i in range(len(destinations)):
           g = geod.Inverse(destinations[i].waypoint.lat, destinations[i].waypoint.long,
                                end.lat, end.long)
           distance = g['s12'] / 1852
           speed=self.vessel.speed
           time = distance / speed
           wpi = self.waypoint_info(distance, time, 0)
           matrix[i][len(destinations)]=wpi
           matrix[len(destinations)][i]=wpi
        return np.array(matrix)
    def calc_relative_heading(self,wave_heading,vessel_heading):
        '''
        calculates relative heading(0-360) between a given wave and vessel heading
        --------------------------------------------------------------------------
        Parameters
        --------------------------------------------------------------------------
        wave_heading: float, heading angle of wave
        vessel_heading: float,heading angle of vessel
        --------------------------------------------------------------------------
        Returns
        --------------------------------------------------------------------------
        float(0-360), relative heading between wave and vessel
        '''
        heading=180+vessel_heading-wave_heading
        if(heading<0):
         heading+=360
        return heading
    
if __name__ == '__main__':
   destinations3 = [Destination(Waypoint(43.334, -170.2), datetime(2018, 1, 1, 8, 0), 1),
                    Destination(Waypoint(45, -167.71), datetime(2018, 1, 1, 17, 0), 7),
                    Destination(Waypoint(43.334, -167.71), datetime(2018, 1, 2, 2, 0), 9),
                    Destination(Waypoint(45, -170), datetime(2018, 1, 2, 10, 0), 4),Destination(Waypoint(25,25),datetime(2018,1,4,5,0),20),
                    Destination(Waypoint(23.334, -170.2), datetime(2018, 1, 1, 8, 0), 1),
                    Destination(Waypoint(25, -167.71), datetime(2018, 1, 1, 17, 0), 7),
                    Destination(Waypoint(23.334, -167.71), datetime(2018, 1, 2, 2, 0), 9),
                    Destination(Waypoint(25, -170), datetime(2018, 1, 2, 10, 0), 4)
                    ,Destination(Waypoint(40,25),datetime(2018,1,4,5,0),20),Destination(Waypoint(40,20),datetime(2018,1,4,5,0),20),Destination(Waypoint(40,-150),datetime(2018,1,4,5,0),20)]
   destinations4 = [Destination(Waypoint(43.334, -170.2), datetime(2018, 1, 1, 8, 0), 1),
                    Destination(Waypoint(45, -167.71), datetime(2018, 1, 1, 17, 0), 7),
                    Destination(Waypoint(43.334, -167.71), datetime(2018, 1, 2, 2, 0), 9),
                    Destination(Waypoint(45, -170), datetime(2018, 1, 2, 10, 0), 4),Destination(Waypoint(25,-160),datetime(2018,1,4,5,0),20),
                    Destination(Waypoint(23.334, -170.2), datetime(2018, 1, 1, 8, 0), 1),
                    Destination(Waypoint(25, -167.71), datetime(2018, 1, 1, 17, 0), 7),
                    Destination(Waypoint(23.334, -167.71), datetime(2018, 1, 2, 2, 0), 9),
                    Destination(Waypoint(25, -170), datetime(2018, 1, 2, 10, 0), 4)]
   destinations2 = [Destination(Waypoint(43.334, -170.2), datetime(2018, 1, 1, 8, 0), 1),
                    Destination(Waypoint(45, -167.71), datetime(2018, 1, 1, 17, 0), 7),
                    Destination(Waypoint(43.334, -167.71), datetime(2018, 1, 2, 2, 0), 9),
                    Destination(Waypoint(45, -170), datetime(2018, 1, 2, 10, 0), 4),Destination(Waypoint(25,25),datetime(2018,1,4,5,0),20)]
   mission3=Mission(destinations3)
    
   print("Orginal Vessel  waypoints")
   for d in mission3.destinations:
        print((d.waypoint.lat, d.waypoint.long))
   vessel=vesselFuel("Ship", (48, -170),(48,-170), mission3, 100, 1)
   planner = missionPlanner(vessel,datetime(2018, 1, 1, 0, 0),"/home/remecca/core/Ocean/NOAA_NorthAtlantic.nc")
   best_mission=planner.plan_mission()
     
   print("Optimal Mission")
   for d in best_mission.destinations:
       print((d.waypoint.lat, d.waypoint.long))
   print(best_mission.total_priority)