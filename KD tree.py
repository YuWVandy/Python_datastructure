# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 21:38:15 2020

@author: 10624
"""
class KDtreenode:
    def __init__(self, dim, points, parent, root, rec):
        """Set up the KD tree data structure
        args:
            root - a node object, root node in the KD tree
            points - a list of point where the root node has been removed, point is the object
            dim - the dimension of each node
            parent - the parent of the current node
            rec - the coordinates of the rectangle: [[x_min, x_max], [y_min, y_max], ...]
        """
        import numpy as np
        
        ##Assumption: 0 - left and down, 1 - right and up, 0 - x, 1 - y        
        self.node = root
        self.parent = parent
        self.points = points
        self.children = [None, None]
        self.childlist = [[], []]
        self.rec = rec
        
        
        if(parent == None):
            self.depth = 0
        else:
            self.depth = self.parent.depth + 1
        
        self.dim = self.depth%dim
                
        if(self.parent == None): #ROOT - 0, BRANCH - 1, LEAF - 2
            self.type = "ROOT"
            
        elif(len(self.points) == 0):
            self.type = "LEAF"
            
        else:
            self.type = "BRANCH"
            
        self.update_rec()
        self.subdivide(dim)
            
    def update_rec(self):
        """Update the left(down) or right(up) rectangle
        """
        import copy
        
        self.leftrec, self.rightrec = copy.copy(self.rec), copy.copy(self.rec)
        self.leftrec[2*self.dim + 1], self.rightrec[2*self.dim] = self.node.dimension[self.dim], self.node.dimension[self.dim]
        
    def subdivide(self, dim):
        import numpy as np
        
        if self.type == "LEAF":
            return
        
        for i in range(len(self.points)):
            if(self.points[i].dimension[self.dim] < self.node.dimension[self.dim]):
                self.childlist[0].append(self.points[i])
            else:
                self.childlist[1].append(self.points[i])
        
        if(len(self.childlist[0]) != 0):
            root0 = self.childlist[0][np.random.randint(len(self.childlist[0]))]
            self.childlist[0].remove(root0)
            self.children[0] = KDtreenode(dim, self.childlist[0], self, root0, self.leftrec)
            
        if(len(self.childlist[1]) != 0):
            root1 = self.childlist[1][np.random.randint(len(self.childlist[1]))]
            self.childlist[1].remove(root1)
            self.children[1] = KDtreenode(dim, self.childlist[1], self, root1, self.rightrec)

def traverseTDtree(point, v_assign, dist, dist_type, ROOT):
    """
    Parameters
    ----------
    point     : vehicle object, with x, y, ID attributes
    dist      : the current minimium distance
    dist_type : the type of the distance we focus on: Euclidean, Weighted Euclidean
    ROOT      : the current building node we are looking at right now
    
    Returns
    -------
    v_assign: 1-D numpy array of dimension the number of vehicles
    """
    
    if(dist_type == "Euclidean"):
        temp_dist = Euclideandist1(point.x, point.y, ROOT.node.x, ROOT.node.y)    
        
    if(dist_type == "Weighted Euclidean"):
        temp_dist = 1/ROOT.node.arearatio*Euclideandist1(point.x, point.y, ROOT.node.x, ROOT.node.y) 
        
    if(temp_dist < dist):
        dist = temp_dist
        v_assign[point.ID] = ROOT.node.ID
    
    #left children or right children?
    if(point.dimension[ROOT.dim] < ROOT.node.dimension[ROOT.dim]):
        if(ROOT.children[0] != None):
            v_assign, dist = traverseTDtree(point, v_assign, dist, dist_type, ROOT.children[0])
            
        if(ROOT.children[1] != None):
            dist_recp = dist_recpoint(point, ROOT.rightrec) #May change in highdimension
            if(dist_recp <= dist):
                v_assign, dist = traverseTDtree(point, v_assign, dist, dist_type, ROOT.children[1])
    else:
        if(ROOT.children[1] != None):
            v_assign, dist = traverseTDtree(point, v_assign, dist, dist_type, ROOT.children[1])
            
        if(ROOT.children[0] != None):
            dist_recp = dist_recpoint(point, ROOT.leftrec)
            if(dist_recp <= dist):
                v_assign, dist = traverseTDtree(point, v_assign, dist, dist_type, ROOT.children[0])
    
    return v_assign, dist

def dist_recpoint(point, rec):
    """Calculate the distance between a point and a rectangle, very general can be used many ways
    Input:
        point - a vehicle node object, with attributes: x, y, ID
        rec - a 2-D list, [x_min, x_max, y_min, y_max]
    """
    
    dx = max(rec[0] - point.x, 0, point.x - rec[1])
    dy = max(rec[2] - point.y, 0, point.y - rec[3])
    
    return (dx**2 + dy**2)**0.5




