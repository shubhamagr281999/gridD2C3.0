#call assigner.assign() to get a new delivery loaction

import pandas as pd

class destination_assign:
	def __init__(self):
		self.data = pd.read_excel (r'~/catkin_ws/src/gridD2C3.0/Sample Data.xls')
		self.destination= self.data['Destination'].tolist()
		self.Total_destination = len(self.destination)
	
	def assign(self):
		self.temp = self.destination[0]
		self.destination.pop(0)
		print(self.temp)
		

if __name__ == '__main__':
	assigner = destination_assign()
	for i in range(300):
		assigner.assign()