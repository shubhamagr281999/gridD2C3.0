#!/usr/bin/env python

import rospy
import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs
from itertools import combinations
from copy import deepcopy
from bot_control.msg import StartGoal, CompletePlan, PathArray
from geometry_msgs.msg import Point
from a_star import AStar

import numpy as np

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location, direction):
        self.time = time
        self.location = location
        self.direction = direction
    def __eq__(self, other):
        # return self.time == other.time and self.location == other.location
        return (self.time == other.time or abs(self.time-other.time)==1) and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y) + str(self.direction))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y, self.direction))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time_1 = -1
        self.time_2 = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time_1) + ',' + str(self.time_2) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])

class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait action
        # Up action
        if state.direction == 0:
            n = State(state.time + 1, Location(state.location.x, state.location.y+1), 0)
            if self.state_valid(n) and self.transition_valid(state, n):
                neighbors.append(n)
        # Down action
        if state.direction == 2:
            n = State(state.time + 1, Location(state.location.x, state.location.y-1), 2)
            if self.state_valid(n) and self.transition_valid(state, n):
                neighbors.append(n)
        # Left action
        if state.direction == 3:
            n = State(state.time + 1, Location(state.location.x-1, state.location.y), 3)
            if self.state_valid(n) and self.transition_valid(state, n):
                neighbors.append(n)
        # Right action
        if state.direction == 1:
            n = State(state.time + 1, Location(state.location.x+1, state.location.y), 1)
            if self.state_valid(n) and self.transition_valid(state, n):
                neighbors.append(n)

        n = State(state.time + 1, state.location, state.direction)
        if self.state_valid(n):
            neighbors.append(n)
            n = State(state.time + 2, state.location, (state.direction+1)%4)
            # neighbors.append(n)
            # n = State(state.time + 2, state.location, (state.direction+2)%4)
            neighbors.append(n)
            n = State(state.time + 2, state.location, (state.direction+3)%4)
            neighbors.append(n)
        
        return neighbors


    def get_first_conflict(self, solution):
        result = Conflict()
        i = 0
        for agent_1, agent_2 in combinations(solution.keys(), 2):
            for i in range(len(solution[agent_1])):
                state_1 = self.get_state(agent_1, solution, i)
                for j in range(len(solution[agent_2])):
                    state_2 = self.get_state(agent_2, solution, j)
                    if state_1 == state_2:
                        result.time_1 = state_2.time
                        result.time_2 = state_2.time
                        result.type = Conflict.VERTEX
                        result.location_1 = state_1.location
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        return result
                    if state_1.is_equal_except_time(state_2) and state_1.time > state_2.time:
                        state_2a = self.get_state(agent_2, solution, j+1)
                        if state_1.is_equal_except_time(state_2a) and (state_1.time < state_2a.time or state_2.time == state_2a.time):
                            result.time_1 = state_2.time
                            result.time_2 = state_2a.time
                            result.type = Conflict.VERTEX
                            result.location_1 = state_1.location
                            result.agent_1 = agent_1
                            result.agent_2 = agent_2
                            return result
            for i in range(len(solution[agent_2])):
                state_1 = self.get_state(agent_2, solution, i)
                for j in range(len(solution[agent_1])):
                    state_2 = self.get_state(agent_1, solution, j)
                    if state_1 == state_2:
                        result.time_1 = state_2.time
                        result.time_2 = state_2.time
                        result.type = Conflict.VERTEX
                        result.location_1 = state_1.location
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        return result
                    if state_1.is_equal_except_time(state_2) and state_1.time > state_2.time:
                        state_2a = self.get_state(agent_1, solution, j+1)
                        if state_1.is_equal_except_time(state_2a) and (state_1.time < state_2a.time or state_2.time == state_2a.time):
                            result.time_1 = state_2.time
                            result.time_2 = state_2a.time
                            result.type = Conflict.VERTEX
                            result.location_1 = state_1.location
                            result.agent_1 = agent_1
                            result.agent_2 = agent_2
                            return result

        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            constraint = Constraints()
            for i in range(conflict.time_1-1, conflict.time_2+2):
                v_constraint = VertexConstraint(i, conflict.location_1)
                constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent[1][0], agent[1][1]), agent[1][2])
            goal_state = State(0, Location(agent[2][0], agent[2][1]), agent[2][2])

            self.agent_dict.update({agent[0]:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self):
        solution = {}
        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent)
            if not local_solution:
                return False
            solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()
    def search(self):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        while self.open_set:
            P = min(self.open_set)
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            if not conflict_dict:
                print("solution found")

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y, 'd':str(state.direction)} for state in path]
            plan[agent] = path_dict_list
        return plan


class wrapper:
    def __init__ (self):
        self.start_goal_agent_sub=rospy.Subscriber("/start_goal_agents", StartGoal, self.start_goal_callback)
        self.cbs_plan_pub=rospy.Publisher("/cbs/plan",CompletePlan,queue_size=1)
        self.start_x=[]
        self.start_y=[]
        self.start_d=[]
        self.goal_x=[]
        self.goal_y=[]
        self.goal_d=[]
        self.bot_num=[]
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("param", help="input file containing map and obstacles")
        self.parser.add_argument("output", help="output file with the schedule")
        self.args = self.parser.parse_args()
        # self.main()
    def main2(self):
        # Read from input file
        with open(self.args.param, 'r') as param_file:
            try:
                param = yaml.load(param_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        dimension = param["map"]["dimensions"]
        obstacles = param["map"]["obstacles"]
        agents = []
        n_agents=len(self.bot_num)
        for i in range(n_agents):
            agents.append([self.bot_num[i],[self.start_x[i],self.start_y[i],self.start_d[i]],[self.goal_x[i],self.goal_y[i],self.goal_d[i]]])
        env = Environment(dimension, agents, obstacles)
        # print(agents)
        # Searching
        cbs = CBS(env)
        solution = cbs.search()
        if not solution:
            print(" Solution not found" )
            return
        list1_init=PathArray()
        msg=CompletePlan()
        print(solution)

        for i in self.bot_num:
            n_states=len(solution[i])
            path_agenti=[]
            for j in range(n_states):
                k=Point()
                k.x=solution[i][j]['x']
                k.y=solution[i][j]['y']
                k.z=float(solution[i][j]['d'])
                path_agenti.append(k)
            temp_pathi=PathArray()
            temp_pathi.statei=path_agenti
            temp_pathi.bot_num=i
            msg.agent.append(temp_pathi)
            
        self.cbs_plan_pub.publish(msg)
        # Write to output file
        with open(self.args.output, 'r') as output_yaml:
            try:
                output = yaml.load(output_yaml, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
                
        output["schedule"] = solution
        output["cost"] = env.compute_solution_cost(solution)

        with open(self.args.output, 'w') as output_yaml:
            yaml.safe_dump(output, output_yaml)
    def start_goal_callback(self,data):
        self.start_x=data.start_x
        self.start_y=data.start_y
        self.start_d=data.start_d
        self.goal_x=data.goal_x
        self.goal_y=data.goal_y
        self.goal_d=data.goal_d
        self.bot_num=data.bot_num
        print('here')
        self.main2()
    

if __name__ == "__main__":
    rospy.init_node("cbs")
    rospy.loginfo("CBS node created")
        
    obj=wrapper()
    rospy.spin()
