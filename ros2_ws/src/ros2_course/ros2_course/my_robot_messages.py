# Define Turtle message as a Python class
class Turtle:
    def __init__(self):
        self.name = ""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

# Define TurtleArray message as a Python class
class TurtleArray:
    def __init__(self):
        self.turtles = []

# Define CatchTurtle service request and response as Python classes
class CatchTurtleRequest:
    def __init__(self):
        self.name = ""

class CatchTurtleResponse:
    def __init__(self):
        self.success = False
