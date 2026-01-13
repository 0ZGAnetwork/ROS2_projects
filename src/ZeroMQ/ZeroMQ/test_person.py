import person_pb2

# new person
p = person_pb2.Person()
p.name = "Pawel Ozga"
p.id = 266078

quat = person_pb2.Quaternion()
quat.x = 1.0
quat.y = 2.0
quat.z = 3.0
quat.w = 5.0

pos = person_pb2.Position()
pos.x = 10.0
pos.y = 5.0

robot = person_pb2.Robot()
robot.position.CopyFrom(pos)
robot.quaternion.CopyFrom(quat)

#  output
print("Name:", p.name)
print("ID:", p.id)
print("Position:", pos)
print("Quaternion:", quat)
print("Robot:", robot)
print("Robot, only z:", robot.quaternion.z)  # default 0.0

# create binary file
with open("person.bin", "wb") as f:
    f.write(p.SerializeToString())

# display file
p2 = person_pb2.Person()
with open("person.bin", "rb") as f:
    p2.ParseFromString(f.read())
    
print("Loaded from file:", p2)
