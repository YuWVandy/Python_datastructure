# -*- coding: utf-8 -*-
"""
Created on Mon Jun 29 21:38:15 2020

@author: 10624
"""
class Node:
    def __init__(self, parent, pointv, pointb, recxy, i, c, leafnode):
        """Set up the class of each node in quadra tree
        args:
            parent - a node class, the parent of the current node
            pointv, pointb - two lists of objects, vehicle and building points contained in current node
            recxy - [xmin, xmax, ymin, ymax]
        """
        
        self.parent = parent
        self.children = [None, None, None, None]
        self.recxy = recxy
        #lower left - 0, lower right - 1, upper left - 2, upper right - 3
        self.direction = i
        
        if(parent == None):
            self.depth = 0
        else:
            self.depth = parent.depth + 1
        
        self.pointv, self.pointb = pointv, pointb
        
        if(self.parent == None): #ROOT - 0, BRANCH - 1, LEAF - 2
            self.type = "ROOT"
            
        elif(min(len(pointv), len(pointb)) < c):
            self.type = "LEAF"
            leafnode.append(self)
        else:
            self.type = "BRANCH"
        
        self.subdivide(c, leafnode)
        
    
    def subdivide(self, c, leafnode):
        from shapely.geometry import LineString
        
        if self.type == "LEAF":
            return
        
        hx, hy = (self.recxy[1] - self.recxy[0])/2, (self.recxy[3] - self.recxy[2])/2
        rects = []
        #lower left - 0, lower right - 1, upper left - 2, upper right - 3
        rects.append([self.recxy[0], self.recxy[0] + hx, self.recxy[2], self.recxy[2] + hy])
        rects.append([self.recxy[0] + hx, self.recxy[1], self.recxy[2], self.recxy[2] + hy])
        rects.append([self.recxy[0], self.recxy[0] + hx, self.recxy[2] + hy, self.recxy[3]])
        rects.append([self.recxy[0] + hx, self.recxy[1], self.recxy[2] + hy, self.recxy[3]])
        
        line_horizon = LineString([(self.recxy[0], self.recxy[2] + hy), (self.recxy[1], self.recxy[2] + hy)])
        line_vertical = LineString([(self.recxy[0] + hx, self.recxy[2]), (self.recxy[0] + hx, self.recxy[3])])
        
        subpointv = [[], [], [], []]
        subpointb = [[], [], [], []]
        
        for v in self.pointv:
            for i in range(len(rects)):
                rect = rects[i]
                if(rect[0] <= v.x and v.x <= rect[1] and rect[2] <= v.y and v.y <= rect[3]):
                    subpointv[i].append(v)
        
        for b in self.pointb:
            if(line_horizon.intersects(b.polygon) and line_vertical.intersects(b.polygon)):
                subpointb[0].append(b)
                subpointb[1].append(b)
                subpointb[2].append(b)
                subpointb[3].append(b)
                
            elif(line_horizon.intersects(b.polygon)):
                if(b.x <= self.recxy[0] + hx):
                    subpointb[0].append(b)
                    subpointb[2].append(b)
                else:
                    subpointb[1].append(b)
                    subpointb[3].append(b)
                    
            elif(line_vertical.intersects(b.polygon)):
                if(b.y <= self.recxy[2] + hy):
                    subpointb[0].append(b)
                    subpointb[1].append(b)
                else:
                    subpointb[2].append(b)
                    subpointb[3].append(b)
            else:
                for i in range(len(rects)):
                    rect = rects[i]
                    if(rect[0] <= b.x and b.x <= rect[1] and rect[2] <= b.y and b.y <= rect[3]):
                        subpointb[i].append(b)

        
        for i in range(len(rects)):
            self.children[i] = Node(self, subpointv[i], subpointb[i], rects[i], i, c, leafnode)




