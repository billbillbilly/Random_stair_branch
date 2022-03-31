import rhinoscriptsyntax as rs
import ghpythonlib.components as ghc
import scriptcontext as sc
import random
import math
import sys

# reference: Relative neighborhood graph - https://blog.schawe.me/relative-neighborhood-graph.html

class stairSystem():
    def __init__(self, intX = None, intY = None, interval = None, elevation = None, num = None, multiple_start_point = False):
        self.x = intX
        self.y = intY
        self.interval = interval
        self.elevation = elevation
        self.num = num
        self.multipleStart = multiple_start_point

    # set up point matrix
    def pointMatrix(self):
        # point matrix that provides x,y coordinates
        self.pointDic = {}
        # values that correspond point matrix to validate if the neighbor is locked ot not
        self.valDic = {}
        # collect rendered point matrix
        pt_list = []
        # collect attractors
        self.attractors = []

        for i in range(self.x):
            for j in range(self.y):
                self.pointDic[(i,j)] = (i*self.interval, j*self.interval, 0)
                # 0 means the point is not locked
                self.valDic[(i,j)] = 0
                # render each point in rhino space
                pt_list += [rs.AddPoint((i*self.interval, j*self.interval, 0))]
        # allow users to pick up a point/several points from point matrix as (a) start point(s)
        if self.multipleStart:
            self.original = []
            pts = rs.GetPoints(message1 = "select points as start points", max_points = 2)
            for each in pts:
                self.original += [rs.CreatePoint(each)]
        else:
            pt = rs.GetPoint("select one point in matrix")
            self.original = rs.CreatePoint(pt)

        # select attractor points
        attractors = rs.GetPoints(message1 = "select points as attractors", max_points = 10)
        for each in attractors:
            pt = rs.CreatePoint(each)
            self.attractors += [(pt[0], pt[1], pt[2])]
        rs.HideObjects(pt_list)
        return self

    # find the position of selected origin
    def origin(self):
        for each in self.pointDic:
            x_ = self.pointDic[each][0]
            y_ = self.pointDic[each][1]
            if self.multipleStart:
                for i in range(len(self.original)):
                    if x_ == self.original[i][0] and y_ == self.original[i][1]:
                        self.original[i] = each
            else:
                if x_ == self.original[0] and y_ == self.original[1]:
                    self.original = each
        return self

    # indentify positions to setup attractor points for creating
    # a gradient of brach-out possibility
    def attractor(self):
        for each in self.pointDic:
            x_ = self.pointDic[each][0]
            y_ = self.pointDic[each][1]
            for i in range(len(self.attractors)):
                if x_ == self.attractors[i][0] and y_ == self.attractors[i][1]:
                    self.attractors[i] = each
        return self

    # detect existing neighbors of a given position
    def detectNeighbor(self, position, matrix_value, drop_postion):
        # set up four different neighbors
        front = (position[0],position[1]+1)
        back = (position[0],position[1]-1)
        left = (position[0]-1,position[1])
        right = (position[0]+1,position[1])
        neighbors = [front, back, left, right]
        # set up conditions to ensure if the position is on the edges of matrix
        if position[0] == 0:
            neighbors.remove(left)
        if position[0] == self.x - 1:
            neighbors.remove(right)
        if position[1] == 0:
            neighbors.remove(back)
        if position[1] == self.y - 1:
            neighbors.remove(front)
        # select unlocked neighbors
        available_neighbors = []
        for each in neighbors:
            if matrix_value[each] == 0:
                available_neighbors += [each]
        # remove the position, where a step already exists
        if drop_postion != None:
            if drop_postion in available_neighbors:
                available_neighbors.remove(drop_postion)
        return available_neighbors

    # set up how likely the stair will be branched out
    def setPossibility(self, possibility):
        split_num = int(round(possibility * 10))
        index = random.sample(range(10), split_num)
        self.if_list = range(10)
        for i in range(10):
            if i in index:
                # 1 is for branching out. 0 is for keeping one neighbor
                self.if_list[i] = 1
            else:
                self.if_list[i] = 0
        return self

    # randomly make a decision if branch out the stair or not based on possibility
    def ifBranchOut(self, position, matrix_value, point, length, factor, step_population):
        # find neighbors
        if point in step_population:
            drop_postion = step_population[point]
        else:
            drop_postion = None
        neighbors = self.detectNeighbor(position, matrix_value, drop_postion)
        # make a decision whether branch out stair or not
        if_branch = random.choice(self.if_list)

        # if the position is the attractors, branch out will be disabled
        if position in self.attractors:
            if_branch = 0

        # if legnth of stair trial cannot be divided by factor, disable branch out
        if length % factor != 0:
            if_branch = 0

        # when if_branch is true and there are more than 2 available neighbors,
        # the stair will be branched out
        if if_branch == 1 and len(neighbors) >= 2:
            selected_neighbor = random.sample(neighbors, 2)
        else:
            if neighbors != []:
                selected_neighbor = random.choice(neighbors)
            else:
                selected_neighbor = neighbors
        return selected_neighbor

    # start to generate stair
    def update(self, step_node_population, step_population, step_list):
        if self.multipleStart:
            for pt in self.original:
                stairBranch(pt[0], pt[1], 0, self.interval, self.num, 0, self.pointDic, self.valDic, self.ifBranchOut,
                            self.elevation).generateStairs(step_node_population, step_population, step_list)
        else:
            stairBranch(self.original[0], self.original[1], 0, self.interval, self.num, 0, self.pointDic, self.valDic, self.ifBranchOut,
                        self.elevation).generateStairs(step_node_population, step_population, step_list)

# this class is for recursively generating or branching out stairs
class stairBranch():
    def __init__(self, x, y, hight, interval, times, num, matrix, matrix_value, branch_function, elevation, previous_position = None):
        self.original_position = (x, y)
        self.current_position = None
        self.previous_position = previous_position
        self.hight = hight
        self.interval = interval
        self.recursion_times = times
        self.pointDic = matrix
        self.valDic = matrix_value
        self.ifBranchOut = branch_function
        self.elevation = elevation
        self.count = num
        self.curveID = None
        self.trailPts = []
        self.folder = '/Users/yangxiaohao/Desktop/RhinoScript/assignment/final/animation/'

    # generate stair step along the trail
    def stepNode(self, point, elevation, interval, step_node_population, node_list):
        if (point[0], point[1], point[2]) not in node_list:
            points = []
            for i in range(2):
                if i == 0:
                    factor1 = 1
                else:
                    factor1 = -1
                for j in range(4):
                    x = point[0] + interval * math.cos(math.radians(90*j))
                    y = point[1] + interval * math.sin(math.radians(90*j))
                    z = point[2] + factor1 * elevation/1.5
                    points += [rs.CreatePoint((x,y,z))]
            box = rs.AddBox(points)
            scale = random.uniform(0.5, 1)
            rs.ScaleObject(box, point, (scale, scale, 0.5))
            rs.RotateObject(box, point, 45)

            # get vertices of modefied box
            # explore the box
            surfaces = rs.ExplodePolysurfaces(box)

            # get two surfaces that are not attached to each other
            areas = []
            areas_dic = {}
            for srf in surfaces:
                area = round(rs.SurfaceArea(srf)[0])
                areas += [area]
                areas_dic[area] = srf
            area = max(areas)
            srf1 = areas_dic[area]
            srf2 = None
            for each in surfaces:
                if srf2 == None:
                    if round(rs.SurfaceArea(srf1)[0]) == round(rs.SurfaceArea(each)[0]) and each != srf1:
                        srf2 = each
            srf = [srf1, srf2]
            # save eight vertexes of surface
            vertices = []
            for each in srf:
                # get edges of surface
                edges = rs.DuplicateEdgeCurves(each)
                    # get vertexes from each edge
                for edge in edges:
                    if rs.CurveEndPoint(edge)[2] == point[2] + elevation/1.5:
                        # save each vertice
                        vertices += [nodeNetwork(rs.CurveEndPoint(edge), interval, 0)]
                    else:
                        vertices += [nodeNetwork(rs.CurveEndPoint(edge), interval, 1)]
                    #rs.AddPoint(rs.CurveEndPoint(edge))
                rs.DeleteObjects(edges)
            node_list += [(point[0], point[1], point[2])]
            step_node_population += vertices
            rs.DeleteObjects(surfaces)
            animate(self.folder, len(node_list))

    # create step and vertices
    def populateStep(self, trails, interval, step_node_population, node_list):
        for trail in trails:
            length = rs.CurveLength(trail)
            pts = rs.DivideCurve(trail, round(length/3))
            delta_elevation = abs(rs.CurveEndPoint(trail)[2] - rs.CurveStartPoint(trail)[2])
            for pt in pts[1:]:
                self.stepNode(pt, delta_elevation/len(pts), math.sqrt(self.interval), step_node_population, node_list)

    # generate stair trials
    def generateStairs(self, step_node_population, step_population, node_list):
        # set origin as start point
        if self.current_position == None:
            self.current_position = self.original_position
            # add point into trail point list
            pt = rs.CreatePoint(self.pointDic[self.original_position])
            pt = rs.CreatePoint((pt[0],pt[1],self.hight))
            # count the number of steps in the indivual branch
            self.trailPts += [pt]
            # save occupied point in a global directionary
            step_population[(pt[0],pt[1],pt[2])] = self.current_position

        # calcualate legnth of stair trail
        length = len(self.trailPts)
        factor = self.elevation*2
        # get spatial point in the current position
        currentPt = (self.pointDic[self.current_position][0],
                     self.pointDic[self.current_position][1],
                     self.hight)
        # find unlocked neighbors and randomly celect one or two neighbors
        neighbors = self.ifBranchOut(self.current_position, self.valDic, currentPt, length, factor, step_population)
        # after getting the unlocked neighbors, unlock previous position
        if self.previous_position != []:
            self.valDic[self.previous_position] = 0
            # if there is no available neighbor, unlock all neighbors
            if neighbors != []:
                for each in self.valDic:
                    self.valDic[each] = 0

        # lock current_position
        self.valDic[self.current_position] = 1
        # save current_position as previous_position
        self.previous_position = self.current_position

        # change the vertical direction of stairs
        step = self.hight/self.elevation
        direction = math.sin(self.count/(self.recursion_times/(self.elevation**2)))
        # go up
        if direction >= 0:
            direction = 1
        # go down
        elif direction < 0 and self.hight >= 0:
            direction = -1
            # when the stair is gonging down, disable the function of branch
            if isinstance(neighbors, list) == True:
                neighbors = random.choice(neighbors)
        # go up
        else:
            direction = 1
        # change elevations
        self.hight += self.elevation * direction

        if neighbors != []:
            # keep generating
            if isinstance(neighbors, list) == False:
                # update current_position by randomly picking up a neighbor
                self.current_position = neighbors
                # create point and add it into list
                x = self.pointDic[self.current_position][0]
                y = self.pointDic[self.current_position][1]
                z = self.hight

                pt = rs.CreatePoint((x,y,z))
                self.trailPts += [pt]
                if self.curveID != None:
                    rs.DeleteObject(self.curveID)
                self.curveID = rs.AddCurve(self.trailPts,1)
                rs.HideObject(self.curveID)
                # explore trial
                trails = rs.ExplodeCurves(self.curveID)
                #rs.HideObjects(trails)
                # generate steps
                self.populateStep(trails, self.interval, step_node_population, node_list)
                rs.DeleteObjects(trails)

                step_population[(x,y,z)] = [self.current_position]
                # recursion
                while self.count <= self.recursion_times:
                    self.count += 1
                    self.generateStairs(step_node_population, step_population, node_list)
            # branch out
            else:
                # create point and add it into list
                #rs.AddCurve(self.trailPts, degree=1)
                for each in neighbors:
                    x = self.pointDic[each][0]
                    y = self.pointDic[each][1]
                    z = self.hight
                    pt = rs.CreatePoint((x,y,z))
                    self.trailPts += [pt]
                    if self.curveID != None:
                        rs.DeleteObject(self.curveID)
                    self.curveID = rs.AddCurve(self.trailPts, 1)
                    #rs.HideObject(self.curveID)
                    # explore trial
                    trails = rs.ExplodeCurves(self.curveID)
                    #rs.HideObjects(trails)
                    # generate steps
                    self.populateStep(trails, self.interval, step_node_population, node_list)
                    self.trailPts.pop(-1)
                    rs.DeleteObjects(trails)

                    step_population[(x,y,z)] = [each]
                if self.count <= self.recursion_times:
                    # generate new stair branches
                    stair1 = stairBranch(neighbors[0][0], neighbors[0][1], self.hight,
                                         self.interval, self.recursion_times, self.count,
                                         self.pointDic, self.valDic, self.ifBranchOut,
                                         self.elevation, self.previous_position)
                    stair2 = stairBranch(neighbors[1][0], neighbors[1][1], self.hight,
                                         self.interval, self.recursion_times, self.count,
                                         self.pointDic, self.valDic, self.ifBranchOut,
                                         self.elevation, self.previous_position)
                    # recursion
                    stair1.generateStairs(step_node_population, step_population, node_list)
                    stair2.generateStairs(step_node_population, step_population, node_list)

# generate stair connections
class nodeNetwork():
    def __init__(self, node, interval, upordown):
        self.pos = node
        self.interval = interval
        self.up_down = upordown
        self.connect_times = 0
        self.connected = []

    # connect neighbors
    def connectNeighbors(self, nodePopulation, connections):
        # check if two points are nearest neighbors
        for i in range(len(nodePopulation)):
            d = rs.Distance(self.pos, nodePopulation[i].pos)
            if d <= self.interval*2:
                for j in range(len(nodePopulation)):
                    distToSelf = rs.Distance(nodePopulation[j].pos, self.pos)
                    distToNode = rs.Distance(nodePopulation[j].pos, nodePopulation[i].pos)
                    if max(distToSelf, distToNode) < d:
                        # this node is in the lune and blocks
                        break
                else:
                    # prevent creating duplicate Curves
                    if self.pos not in nodePopulation[i].connected:
                        connections += [rs.AddCurve((self.pos, nodePopulation[i].pos), 1)]
                        # label nodes as connected nodes and save them
                        self.connected += [nodePopulation[i].pos]
                        nodePopulation[i].connected += [self.pos]
                        # count times of connections
                        self.connect_times += 1
                        nodePopulation[i].connect_times += 1
        return self

    # calculate horizontal distance between two vertices
    def horizontal_dist(self, pt1, pt2):
        return ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**0.5

    # vertically connect vertices
    def verticalConnect(self, nodePopulation, connections):
        for i in range(len(nodePopulation)):
            # only connect lower vertices to up vertices of other steps
            if self.up_down == 1 and nodePopulation[i].up_down == 0 and self.pos[2] > nodePopulation[i].pos[2]:
                if abs(self.pos[2] - nodePopulation[i].pos[2]) >= 0.5*self.interval:
                    horizontal_distance = self.horizontal_dist(self.pos, nodePopulation[i].pos)
                    if horizontal_distance <= math.sqrt(self.interval) and self.pos not in nodePopulation[i].connected:
                        if nodePopulation[i].connect_times <= 3:
                            connections += [rs.AddCurve((self.pos, nodePopulation[i].pos), 1)]
                            # label nodes as connected nodes and save them
                            self.connected += [nodePopulation[i].pos]
                            nodePopulation[i].connected += [self.pos]
                            # count times of connections
                            self.pos += 1
                            nodePopulation[i].connect_times += 1
        return self

    def connect(self, nodePopulation, connections):
        self.connectNeighbors(nodePopulation, connections)
        self.verticalConnect(nodePopulation, connections)

def animate(folder, sequence_num):
    f_path = folder + "{}.png".format(int(sequence_num))
    rs.Command("_-ViewCaptureToFile " + f_path + " _Enter")

def main():
    folder = '/your/folder/'
    # get inpit
    intX = rs.GetInteger("X", 10)
    intY = rs.GetInteger("Y", 10)
    interval = rs.GetInteger("set interval between each point", 10)
    if_multiple = rs.GetInteger("setup two start points = 1 or single start point = 0", 0)
    elevation = rs.GetInteger("set elevation", 3)
    possibility = rs.GetReal("set possibility (from 0.1 to 0.9) of branching out stairs",
                             0.4, minimum = 0.1, maximum = 0.9)
    recursion_num = rs.GetInteger("set iteration of recursion", 10)

    # convert if_multiple to Boolean
    if if_multiple == 0:
        if_multiple = False
    elif if_multiple == 1:
        if_multiple = True
    else:
        sys.exit('wrong number, input should be 0 or 1')

    # list of vertices of each step
    step_node_population = []
    # list of positions of each step
    step_list = []
    # dictionary of placed positions of steps
    step_population = {}
    # list of connections between steps
    connections = []
    # generate stairs
    stairs = stairSystem(intX, intY, interval, elevation, recursion_num, if_multiple).pointMatrix().origin().attractor().setPossibility(possibility)
    stairs.update(step_node_population, step_population, step_list)

    # generate stair connections
    for each in step_node_population:
        each.connect(step_node_population, connections)
    # conver connections into polylines
    polylines = []
    join = ghc.JoinCurves(connections)
    for each in join:
        polylines += [sc.doc.Objects.AddCurve(each)]
    rs.DeleteObjects(connections)
    # group up polylines
    group = rs.AddGroup("polylines")
    rs.AddObjectsToGroup(polylines, group)
    animate(folder, len(step_list)+1)
    # generate solid geometry based on connections
    #commandStr = "_MultiPipe {} 0.25 on Enter 1 Enter".format(polylines)
    #rs.Command(commandStr)

main()
