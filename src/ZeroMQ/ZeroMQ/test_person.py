import person_pb2

# new person
p = person_pb2.Person()
p.name = "Pawel Ozga"
p.id = 266078

#  output
print("Name:", p.name)
print("ID:", p.id)

# create binary file
with open("person.bin", "wb") as f:
    f.write(p.SerializeToString())

# display file
p2 = person_pb2.Person()
with open("person.bin", "rb") as f:
    p2.ParseFromString(f.read())

print("Loaded from file:", p2)
