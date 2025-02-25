from datetime import datetime


class Waypoint:
    '''
    A point in space and time that a vessel must travel through.
    '''
    def __init__(self, lat, long):
        '''
        Waypoint Constructor
        ---------------------
        Parameters
        -------------------
        lat: latitude of waypoint
        long: longitude of waypoint
        '''
        self.lat = lat
        self.long = long

class Destination:
   '''
    A waypoint with a goal arrival time and priority score
   '''
   def __init__(self, waypoint, arrival_time, priority):
      '''
        Destination Constructor
        ---------------------
        Parameters
        -------------------
        waypoint: waypoint object
        arrival_time: goal time for the vessel to arrive at the waypoint
        '''
      self.waypoint=waypoint
      self.arrival_time=arrival_time
      self.priority=priority
      self.visited=False

class Mission:
    '''
    A mission is a list of destinations that a vessel travels through 
    '''
    def __init__(self, destinations):
        '''
        MissionConstructor
        ---------------------
        Parameters
        -------------------
        destinations: a list of destination objects
        current_waypoint_index: the current destination the vessel is traveling to
        total priority: sum of all priority values in a mission(used for optimal mission planning)
    
        '''
        self.destinations = destinations
        self.current_waypoint_index = 0
        self.total_priority=0
        count=0
        for dest in self.destinations:
           count+=1
        
           self.total_priority+=dest.priority
   
    
    def current_waypoint(self):
        '''
        returns the waypoint the vessel is currently heading towards
        '''
        if self.current_waypoint_index < len(self.destinations):
            print(self.destinations[self.current_waypoint_index])
            return self.destinations[self.current_waypoint_index].waypoint
        return None
    def current_goal_time(self):
        '''
        returns the goal time of the waypoint the vessel is currently heading towards
        '''
        if self.current_waypoint_index < len(self.destinations):
            return self.destinations[self.current_waypoint_index].arrival_time
        return None
    def current_priority(self):
        '''
        returns the priority score of the waypoint the vessel is currently heading towards
        '''
        if self.current_waypoint_index < len(self.destinations):
            return self.destinations[self.current_waypoint_index].priority
        return None
   
    def advance_waypoint(self):
        '''
        changes the vessesl's current waypoint to the next one in the list.
        -------------------------------------------------------------------
        Returns
        -------------------------------------------------------------------
        Boolean: false if the current waypoint is the last one in the list, else true
        '''
        if self.current_waypoint_index < len(self.destinations) - 1:
            self.current_waypoint_index += 1
            return True
       
        return False
class vesselBase:
    '''
     Simple base class for vessel objects
    '''
    def __init__(self,name,start,end,mission,speed=0,heading=0,fuel_weight=100):
        '''
        vesselBase constructor
        -------------------------
        Parameters:
        ------------------------
        name: string, vessel name
        start: tuple of (lat, long), vessel starting point
        end: tuple of (lat, long), vessel ending point
        mission: mission object
        speed: float, vessel's starting speed in knots
        heading: float, vessel's starting heading in degrees (0 = north, 90 = east)
        fuel_weight: float, vessel's starting fuel weight in kN
        '''
        self.name = name
        self.start = start 
        self.end=end
        self.mission = mission 
        self.speed = speed 
        self.heading = heading 
        self.fuel_weight=fuel_weight

        end_arrival_time=mission.destinations[len(mission.destinations)-1].arrival_time
        mission.destinations.append(Destination(Waypoint(end[0],end[1]),end_arrival_time,0))
        self.methods = []
class vesselFuel(vesselBase):
    '''
    sub class of vesselBase adds fuel calculations
    '''
    def __init__(self,name,start,end,mission,fuel_weight,speed=0,heading=0):
        '''
        vesselFuel Constructor
        '''
        super().__init__(name,start,end,mission,speed,heading,fuel_weight)
        self.fuel_burned=0 #float-fuel burned in the current time interval(kN)


    def timestep(self, time_interval, current_time=datetime(2018,1,1,1,1)):
        '''
        performs fuel calculations and updates fuelweight 
        -------------------------------------------------
        Parameters
        -------------------------------------------------
        time_interval: float, amount of time that passed in hours
        current_time: datetime object, current time of the simulation
        '''
        self.fuel_burned=self.calc_fuel_burned(self.speed,time_interval)
        self.fuel_weight-=self.fuel_burned
        if(self.fuel_weight<=0):
           self.get_fuel_time(current_time,time_interval,self.fuel_burned)
           self.fuel_weight=0
           self.speed=0
           self.active=False
    
    def calc_fuel_burned(self,speed,time_interval):
        '''
        updates fuel calculation variables for time_interval
        ----------------------------------------------------
        Parameters
        ----------------------------------------------------
        time_interval: float, amount of time that passed in hours
        '''
        #approximated fuel burnt(more complicated calculations done in actual simulation)
        return 0.05*(speed**3)*time_interval