import random
import sys
import os
import time
from copy import deepcopy

"""
defining colors used for display
"""
CBLACK  = '\33[30m'
CWHITE  = '\33[37m'
CWHITEBG = '\033[107m'
CBLUEBG = '\33[104m'
CBLUE = '\33[94m'
CYELLOW = '\33[93m'
CRED = '\33[91m'
CREDBG = '\33[101m'
CGREEN = '\33[92m'
CBOLD   = '\33[1m'
CEND = '\033[0m'
CBLINK = '\033[5m'
CURL = '\033[4m'

"""
class puzzle allows instantiation of a random 
puzzle and drwaing the current state 
"""
class puzzle:
	#init a puzzle with 4 x 3 elements
	def __init__(self):
		self.h=0
		self.g=0
		self.f=0
		#This boolean is to check if the node is part of the closed list
		self.is_in_closed=False
		#This boolean is to check if the node is part of the open list
		self.is_on_open=False
		#parent node needed to retrace the different steps towards the goal after solving the puzzle
		self.parent=None
		"""
		This initialize start to the goal state
		so that we start randomly shufflling the puzzle later
		(moving the empty tile) starting from the goal state
		in order to be sure we are going to start solving a puzzle
		that is in a solvable state(must have a solution)
		"""
		self.current=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]

	#draw and display the puzzle grid
	def draw(self):
		for i in range(0,4):
			for j in range(0,3):
				if self.current[i][j]==0:
					print(CBLUEBG+"	"+"	"+CEND,end='')
				else:
					print(CBLACK+CWHITEBG+"|	"+str(self.current[i][j])+"	"+CEND,end='')
			print("\v")
	
#this function print a list in a matrix form with the 0 value(empty tile)  highlighted:
def drawlist(state):
	#print(" [ ",end='')
	print("")
	for i in range(0,4):
		print(" [ ",end='')
		for j in range(0,3):
			if state[i][j]==0:
				if j==2:
					#print(CBOLD+CBLINK+CRED+str(state[i][j])+CEND,end='')
					print(CREDBG+" "+CEND,end='')
				else:
					#print(CBOLD+CBLINK+CRED+str(state[i][j])+"	"+CEND,end='')
					print(CREDBG+" "+CEND+"	",end='')
			else:
				if j==2:
					print(str(state[i][j]),end='')
				else:
					print(str(state[i][j])+"	",end='')
		#print(" ] ",end='')
		print(" ] ")
	#print(" ] ",end='')
	print('')	

def printa(txt):
	print(txt)

"""	
shuffle randomly with a constrained number of steps
current : the puzzle current state from which the randomization will take place
n : the number of times the empty tile will be moved
"""
def shuffle(current,n):
	#in the main program current will be equal to the puzzle in its goal state
	next=current
	neighbors=[]
	#Loop to move the empty tile n times
	for i in range(0,n):
		#generate neighbors of next node using successor_function
		neighbors=successor_function(next)
		#Choose one of the generated neighbors randomly
		#and then repeat again the same operation for the chosen node until the end of the for loop
		number_of_neighbors=len(neighbors)
		random_pos=random.randint(0,number_of_neighbors-1)
		next=neighbors[random_pos]
	return next
		

#generate successors list
def successor_function(current):
	#get the coordinates of the empty tile
	x,y=get_empty_coord(current)
	successor_list=[]
	#expand the current node and generate the neighbor nodes where the empty tile is able to move
	#depending from x and y the ability for the empty tile to move is determined
	#and so the number of neighbors generated 
	#could be 2, 3 or 4 because the empty tile move horizontally or diagonally 
	#generate neighbor when the empty tile can move to the left
	if x-1>=0 :
		succ=deepcopy(current)
		succ.current[x][y]=succ.current[x-1][y]
		succ.current[x-1][y]=0
		succ.parent=current
		succ.g=float('inf')
		succ.f=float('inf')
		succ.is_on_open=False
		succ.is_in_closed=False
		successor_list.append(succ)
	#generate neighbor when the empty tile can move to the right
	if x+1<=3 :
		succ=deepcopy(current)
		succ.current[x][y]=succ.current[x+1][y]
		succ.current[x+1][y]=0
		succ.parent=current
		succ.g=float('inf')
		succ.f=float('inf')
		succ.is_on_open=False
		succ.is_in_closed=False
		successor_list.append(succ)
	#generate neighbor when the empty tile can move down
	if y-1>=0 :
		succ=deepcopy(current)
		succ.current[x][y]=succ.current[x][y-1]
		succ.current[x][y-1]=0
		succ.parent=current
		succ.g=float('inf')
		succ.f=float('inf')
		succ.is_on_open=False
		succ.is_in_closed=False
		successor_list.append(succ)
	#generate neighbor when the empty tile can move up
	if y+1<=2 :
		succ=deepcopy(current)
		succ.current[x][y]=succ.current[x][y+1]
		succ.current[x][y+1]=0
		succ.parent=current
		succ.g=float('inf')
		succ.f=float('inf')
		succ.is_on_open=False
		succ.is_in_closed=False
		successor_list.append(succ)
	return successor_list


#function that returns the index of the lowest f cost node in a list
def least_f_cost(openlist):
	score=openlist[0].f
	index=0
	for i in range(0,len(openlist)):
		if openlist[i].f<score :
			score=openlist[i].f	
			index=i
	return openlist[index]


"""
This function is the implementation of the A* algorithm used for solving the 4x3 puzzle
"""	
def Astar(start_state):
	goal=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]
	#This variable is used to track the number of expanded nodes or generated neighbors for each checked node
	generated_nodes=1
	"""
	These variables f,g and h are used for determining the cost of any node
	g is the cost of going from a current node to its neighbor node
	h is the heuristic used here (manhattan heuristic) which is the estimated cost to go
	from current node to the goal node (in this program I used manhattan heuristic)
	f is the sum of both : f=g+h
	"""
	start_state.h=Heuristic_3(start_state)
	start_state.g=0	
	start_state.f=start_state.g+start_state.h
	#open list that contain nodes that have been visited but neighbors were not generated yet (thus to be explored)
	start_state.is_on_open=True
	openlist=[start_state]
	while openlist:
		#return the node with the lowest total cost f
		current=least_f_cost(openlist)
		#display the current state of the puzzle grid of that node
		#display as well the f cost , g cost and h cost for the current node
		print("#################################################")
		current.draw()
		print("\v")
		print(" f(n) = ",current.f)
		print(" g(n) = ",current.g)
		print(" h(n) = ",current.h)
		"""
		if the current node has the same order of tiles as the goal node then 
		return the execution time for solving the puzzle as weel as the number of 
		expanded nodes and the current node
		"""
		if current.current == goal :
			return generated_nodes,current 
		#expand current node by generating its neighbors	
		successor_list=successor_function(current)
		#remove the current node from the openlist
		#and put it in the closed list after all neighbors are expanded
		current.is_on_open=False
		openlist.remove(current)
		current.is_in_closed=True

		for state in successor_list:
			#increment the count for generated nodes 
			generated_nodes=generated_nodes+1
			#if we already visited this node and expanded its neighbors then just ignore it
			if state.is_in_closed==True:
				continue

			#each neighbor is obtained by moving the empty tile only by one move
			#this is why we increment g only by adding 1
			g_cost=current.g+1	
			#if not in open list add this node to the openlist to mark it as a node to be expanded`
			if state.is_on_open==False:
				#add to open list
				state.is_on_open=True
				openlist.append(state)
			else:	
				#check if g cost is better 
				if state.g<=g_cost:
					continue
			state.g=g_cost
			##compute the estimated cost h by using heuristic function
			state.h=Heuristic_3(state)
			##total cost f is the sum of g plus h
			state.f=state.g+state.h
			state.parent=current
	if (current.current!=goal):
		print("there is no solution for this puzzle!")
		exit
		
#finding the position (x,y) iof the empty tile on the puzzle
def get_empty_coord(state):
	x=0
	y=0
	for i in range(0,4):
		for j in range(0,3):
			if state.current[i][j]==0:
				x,y=i,j
				break
	return x,y


#Manhattan heuristic
def Heuristic_1(start):
	#final goal node 
	goal=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]
	#This is used to get the indexes of the correct position for any tile in the start node
	indexes=[[0,0],[0,1],[0,2],[1,0],[1,1],[1,2],[2,0],[2,1],[2,2],[3,0],[3,1],[3,2]]
	sum=0
	for i in range(0,4):
		for j in range(0,3):
			#Compute the sum of costs to move tiles to their correct position as denoted by goal matrix 
			if start.current[i][j]!=goal[i][j]:
				#Exclude the empty tile when computing the sum of costs for each tile
				if start.current[i][j]!=0:
					#Get the final goal index for any given i index of the start node and assign it to x
					x=indexes[int(start.current[i][j])-1][0]
					#Get the final goal index for any given j index of the start node and assign it to y
					y=indexes[int(start.current[i][j])-1][1]
					#Sum up the cost to move each position (i,j) to its correct (x,y) coordinates in the puzzle
					sum=sum+(abs(i-x)+abs(j-y))
	return sum


#Heuristic based on the number of misplaced tiles
def Heuristic_2(start):
	goal=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]
	sum=0
	for i in range(0,4):
		for j in range(0,3):
			if start.current[i][j]!=goal[i][j]:
				#don't count empty tile
				if start.current[i][j]!=0:
					sum=sum+1
	return sum	

#Heuristic based on the number of tiles outside of row + number of tiles outside of column
def Heuristic_3(start):
	goal=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]
	#This is used to get the indexes of the correct position for any tile in the start node
	indexes=[[0,0],[0,1],[0,2],[1,0],[1,1],[1,2],[2,0],[2,1],[2,2],[3,0],[3,1],[3,2]]
	sum=0
	for i in range(0,4):
		for j in range(0,3):
			#don't count empty tile
			if start.current[i][j]!=0:
				#Get the final goal index for any given j index of the start node and assign it to y
				y=indexes[int(start.current[i][j])-1][1]
				#Get the final goal index for any given i index of the start node and assign it to x
				x=indexes[int(start.current[i][j])-1][0]
				#tile out of column
				if j!=y:
					sum=sum+1
				#tile out of row 
				if i!=x:
					sum=sum+1
	return sum

#This heuristic function computes the sum of euclidean distances between misplaced tiles and their
#correct position and add 2 for adjacent tiles that need to be swapped.
def Heuristic_4(start):
	#final goal node 
	goal=[[1,2,3],[4,5,6],[7,8,9],[10,11,0]]
	#This is used to get the indexes of the correct position for any tile in the start node
	indexes=[[0,0],[0,1],[0,2],[1,0],[1,1],[1,2],[2,0],[2,1],[2,2],[3,0],[3,1],[3,2]]
	sum=0
	for i in range(0,4):
		for j in range(0,3):
			#Compute the sum of costs to move tiles to their correct position as denoted by goal matrix 
			if start.current[i][j]!=goal[i][j]:
				#Exclude the empty tile when computing the sum of costs for each tile
				if start.current[i][j]!=0:
					#Get the final goal index for any given i index of the start node and assign it to x
					x=indexes[int(start.current[i][j])-1][0]
					#Get the final goal index for any given j index of the start node and assign it to y
					y=indexes[int(start.current[i][j])-1][1]
					#Sum up the cost to move each position (i,j) to its correct (x,y) coordinates in the puzzle
					sum=sum+(abs(i-x)+abs(j-y))
					if j<=1 :
						#check if adjacent horizontal tile is not an empty tile
						if start.current[i][j+1]!=0:
							if start.current[i][j]-start.current[i][j+1]==1 :
								sum=sum+2
					if i<=2 :
						#check if adjacent vertical tile is not an empty tile
						if start.current[i+1][j]!=0:
							if start.current[i][j]-start.current[i+1][j]==3 :
								sum=sum+2
						
	return sum
		

#This function runs an animation of the steps needed to slove the puzzle
def rebuild_steps(solution,start,seconds):
	list_of_steps_to_solution=[]
	
	#retrace back the path from the solution to the start state
	#and save the steps into a list
	parent=solution
	while parent!=start:
		list_of_steps_to_solution.append(parent)
		parent=parent.parent
	list_of_steps_to_solution.append(start)

	for item in reversed(list_of_steps_to_solution):
		#time to refresh the display for each move of the empty tile
		time.sleep(seconds)
		#check which operating system and then clear the screen
		#check if windows os
		if os.name == 'nt':
			os.system('cls')
		#linux and mac
		else:
			os.system('clear')
		#the steps below are to display puzzle matrix steps towards the solution
		print(CYELLOW+" List of steps to solution is :\v"+CEND)
		step_number=0
		#draw matrices representing steps to solution in correct order 
		for items in reversed(list_of_steps_to_solution):
			print(CURL+" Step ",step_number," :"+CEND,end='')
			drawlist(items.current)
			step_number=step_number+1
		print(CYELLOW+" solution steps: \v"+CEND)
		item.draw()
		print("\v")
		print(CYELLOW+" Start state is : \v"+CEND)
		#draw the puzzle of the start state
		start.draw()
	return step_number-1

def main():
	#increase recursion limit that was set by default
	sys.setrecursionlimit(10000)
	#create a new start puzzle object
	start=puzzle()
	#shuffle the starting puzzle by moving empty tile randomly for n times
	shuffled_start=shuffle(start,30)
	
	"""
	Test inputs
	"""
	#25 move shuffle
	#shuffled_start.current=[[1,3,0],[4,2,5],[8,9,6],[7,10,11]]
	#45 move shuffle
	#shuffled_start.current=[[4,1,3],[6,0,9],[2,7,5],[10,8,11]]
	#35 move shuffle
	#shuffled_start.current=[[2,4,1],[6,0,3],[8,5,9],[7,10,11]]
	#100 move shuffle
	#shuffled_start.current=[[1,5,3],[7,4,6],[10,9,2],[8,11,0]]
	#case with adjacent tiles to be swapped 
	#(8,7 and 10,9 are both horizontally and vertically adjacent and positions swapped)
	#shuffled_start.current=[[1,2,3],[4,5,6],[11,10,9],[8,7,0]]
	#80 move shuffle
	#shuffled_start.current=[[4,5,3],[2,6,9],[1,0,11],[7,8,10]]

	#start_time and duration are used to get the execution time of solving the puzzle
	start_time = time.time()
	generated_nodes,solution=Astar(shuffled_start)
	duration=time.time()-start_time
	print(CRED+"BINGO!!!! shortest path has been found!!!!"+CEND);
	print("~~~~~~~~~~~~~")
	print(" Start matrix")
	shuffled_start.draw()
	print("~~~~~~~~~~~~~")
	input(CRED+CBLINK+" Press Enter to play animation that retrace the shortest path to the solution..."+CEND)
	number_of_steps=rebuild_steps(solution,shuffled_start,0)
	print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
	print(CGREEN+" Number of expanded nodes : "+CEND,generated_nodes)
	print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
	print(CRED+" The lowest number of moves to get solution : "+CEND,number_of_steps)
	print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
	print(CYELLOW+" Time needed to solve the puzzle : "+CEND,duration, " seconds \v")
	print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

main()


		
