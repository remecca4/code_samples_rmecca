'''
Finds the fastest path through a group of waypoints using 
a branch and bound algorithm

Author: Rachel Mecca

(c) 2024 Regents of the University of Michigan
'''
from geographiclib.geodesic import Geodesic
import numpy as np
geod = Geodesic.WGS84
maxsize = float('inf')
class tsp_solver:
    '''
    Finds the fastest path through a group of waypoints using 
    a branch and bound algorithm
    '''
    def __init__(self, vessel,dest_list,adj_mat):
        '''
        tsp_solver Constructor
        ------------------------------------------------------------
        Parameters
        ------------------------------------------------------------
        vessel: vessel object
        adj_mat: 2D array of waypoint info objects, the tsp solver will 
        find the fastest route through these objects starting at the 
        first destination in the list
        '''
        self.vessel=vessel
        self.dest_list=dest_list
        self.adj_mat=adj_mat
        self.num_coords=len(dest_list)
        self.fast_sol=self.calc_TSP_fast() #list of indexes of dest_list, approximate the optimal path
        self.best_path=self.fast_sol[0] #list of indexes of dest_list, the optimal path
        self.best_path_length=self.fast_sol[1] # float, total fuel it takes for vessel to travel through best_path
        self.best_path_avg_speed=0 #float, avg speed of vessel toggkrglkrjlkwrihgrr
        self.curr_path=self.best_path #list of indexes of dest_list,current best path    
    class mst_coord:
       '''
       mst coordinate object, used for calc_MST()
       '''
       def __init__(self, index, kv, dw, pw):
            '''
            mst_coord constructor
            ---------------------------------------------------------
            Parameters
            ---------------------------------------------------------
            index: int, index number in dest_list 
            kv:boolean, true if this coordiante has been added to the mst
            dw: float, fuel between index and pw 
            pw: int, index of previous destination 
            '''
            self.index = index
            self.kv = kv
            self.dw = dw
            self.pw=pw
    def make_mst_coords(self,coords):
      '''
      creates a list of mst coords from a list of dest_list indexes
      ----------------------------------------------------------------
      Parameters
      ----------------------------------------------------------------
      coords: list of indexes of dest_list
      Returns
      -----------------------------------------------------------------
      list of mst coords
      '''
      mst_coords=[]
      for i in coords:
        mst_coords.append(self.mst_coord(i,False,maxsize,None))
      mst_coords[0].dw=0
      return mst_coords
    def calc_TSP_fast(self):
     '''
     Greedy random insertion algorithm to quickly find a 
     path that is close to optimal
     ----------------------------------------------------
     Returns
     ----------------------------------------------------
     pair(list of indexes, float), order of dest_list indexes to 
     visit and total fuel used for vessel to travel this path
     '''
     edges=[]
     total_fuel=0
     self.dest_list[0].visited=True
     edges.append(0)
     for v in range(1,self.num_coords):
      best_insert=0
      min_fuel=maxsize
      for i in range(0, len(edges)):
       fuel1=self.adj_mat[v][edges[i]].fuel_burned
       fuel2=0
       fuel3=0
       if( i!=len(edges)-1):
        fuel2=self.adj_mat[v][edges[i+1]].fuel_burned
        fuel3=self.adj_mat[edges[i]][edges[i+1]].fuel_burned
      
       else:
        fuel2=self.adj_mat[v][edges[0]].fuel_burned
        fuel3=self.adj_mat[edges[i]][edges[0]].fuel_burned
      
       if(fuel2+fuel1-fuel3<min_fuel):
        min_fuel=fuel2+fuel1-fuel3
        best_insert=i
      
  
      edges.insert(best_insert+1, v)
 
    
     for e in range(0, len(edges)-1):
      total_fuel+=self.adj_mat[edges[e]][edges[e+1]].fuel_burned
     total_fuel+=self.adj_mat[edges[len(edges)-1]][self.num_coords].fuel_burned
     return(edges,total_fuel)
    def calc_TSP_opt(self):
     '''
     calls recursive genPerms function to find  the 
     optimal path through all destinations in dest_list
     '''
     self.genPerms(1)
    def genPerms(self,permLength):
     '''
     recursive function that uses branch and bound to find the optimal path
     and updates best_path and best_path_length when the optimal path is found
     ---------------------------------------------------------------------
     Parameters
     ---------------------------------------------------------------------
     permLength: int, length of current permutation
     '''
     if permLength == len(self.curr_path):
      total_fuel=0#self.adj_mat[self.curr_path[0]][self.curr_path[len(self.curr_path)-1]].fuel_burned
      for e in range(0, len(self.curr_path)-1):
        total_fuel+=self.adj_mat[self.curr_path[e]][self.curr_path[e+1]].fuel_burned
      total_fuel+=self.adj_mat[self.curr_path[len(self.curr_path)-1]][self.num_coords].fuel_burned
      if(self.best_path_length>total_fuel):
       self.best_path=self.curr_path
       self.best_path_length=total_fuel

      return
     if (not self.promising(self.curr_path, permLength)):
      return
     for i in range(permLength,len(self.curr_path)): 
      temp=self.curr_path[permLength]
      self.curr_path[permLength]=self.curr_path[i]
      self.curr_path[i]=temp
      self.genPerms( permLength + 1)
      temp=self.curr_path[permLength]
      self.curr_path[permLength]=self.curr_path[i]
      self.curr_path[i]=temp
    def promising(self,path,permLength):
     '''
     estimates the total fuel needed for vessel to travel through the 
     unvisited destinations and returns true if the path is promising
     ---------------------------------------------------------------
     Parameters
     ---------------------------------------------------------------
     path: list of indexes of dest_list, possible optimal path
     permLength: int, length of current permutation 
     ---------------------------------------------------------------
     Returns
     --------------------------------------------------------------
      boolean: true if curr_fuel+estimation<best path length
     '''
     curr_unvisited=path[permLength:]
     curr_fuel=0
     if(len(curr_unvisited)<5):
      return True
     for e in range(0,permLength-1):
        curr_fuel+=self.adj_mat[path[e]][path[e+1]].fuel_burned
     
     mst_length=self.calc_MST(curr_unvisited)
     arm1_len=maxsize
     arm2_len=maxsize
     for i in range(0,len(curr_unvisited)):
      dist1=self.adj_mat[path[0]][curr_unvisited[i]].fuel_burned
      dist2=self.adj_mat[path[permLength-1]][curr_unvisited[i]].fuel_burned
      if(dist1<arm1_len):
       arm1_len=dist1
      if(dist2<arm2_len):
       arm2_len=dist2
     min_dist_to_end=np.inf
     for w in curr_unvisited:
       dist=self.adj_mat[w][self.num_coords].fuel_burned
       if(dist<min_dist_to_end):
         min_dist_to_end=dist
     totalEst=curr_fuel+mst_length+arm2_len+arm1_len+min_dist_to_end
     promise=totalEst<self.best_path_length
     return promise
    def calc_MST(self,coords):
     '''
     calculates the length of the minimum spanning tree 
     through coords used for estimation in promising function
     --------------------------------------------------------
     Parameters
     --------------------------------------------------------
     coords: list of indexes of dest_list
     --------------------------------------------------------
     Returns
     --------------------------------------------------------
     float: length of MST
     '''
     curr_v=0
     total_fuel=0
     mst_coords=self.make_mst_coords(coords)
     for c in mst_coords:
        min_fuel=maxsize
        for j in range(0,len(mst_coords)):
          if( not mst_coords[j].kv):
              if(mst_coords[j].dw<min_fuel):
                min_fuel=mst_coords[j].dw
                curr_v=j
              
        mst_coords[curr_v].kv=True

        total_fuel+=mst_coords[curr_v].dw
     
        for j in range(0, len(mst_coords)):
          if(not mst_coords[j].kv ):
              dist=self.adj_mat[mst_coords[curr_v].index][mst_coords[j].index].fuel_burned
              if(dist<mst_coords[j].dw):
                mst_coords[j].dw=dist
                mst_coords[j].pw=curr_v
              
     return total_fuel