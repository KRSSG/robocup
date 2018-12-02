import sys
import PlaySelector
class GUI_link:
	def __init__(self):
		self.path = "RRT"
		self.initialPosition = "Position 1"
		self.team = "Team Yellow"
		self.goalie = "0"
		self.play = "Offensive"
		self.skillchoice = "goToBall"
		self.bot = "0"
	def skill_triggered(self):
		print self.skillchoice, self.bot
	def play_triggered(self):
		print self.path, self.initialPosition, self.team, self.goalie, self.play
	def play_selector_triggered(self):
		PlaySelector.main()
